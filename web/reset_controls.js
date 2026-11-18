const resetControls = (() => {
  const HOLD_MS = 1000;
  const AXIS_ON = 0.68;
  const AXIS_OFF = 0.35;
  const SCOPES = ['right', 'left', 'both'];
  const LABELS = ['右臂', '左臂', '双臂'];

  const state = {
    open: false,
    page: 'menu',
    selected: 0,
    busy: false,
    previous: { y: false, b: false, x: false, a: false },
    ybCandidate: false,
    ybStartedAt: null,
    consumeTail: false,
    ignoreBackUntilRelease: false,
    xStartedAt: null,
    axisLatched: false,
    confirmAArmed: false,
    awaitingPostRequestRelease: false,
    requestInputsReleased: false,
    requestToken: 0,
    execution: null,
    lastRun: null,
    baselineId: null,
    waitingRun: false,
    stopWanted: false,
    stopPending: false,
    stopArmed: {a:false, b:false},
    lastFreshAt: 0,
    closeAt: 0,
    statusMessage: '',
    fallbackOperationId: 0
  };

  function redraw() {
    globalThis.autolifeResetView?.(state.open);
    if (typeof globalThis.drawNavigationPanel === 'function') {
      globalThis.drawNavigationPanel();
    }
  }

  function normalizeButtons(input) {
    return {
      y: Boolean(input.y),
      b: Boolean(input.b),
      x: Boolean(input.x),
      a: Boolean(input.a)
    };
  }

  function areReleased(buttons) {
    return !buttons.y && !buttons.b && !buttons.x && !buttons.a;
  }

  function setPrevious(buttons) {
    state.previous = buttons;
  }

  function getAxisY(axis) {
    if (Array.isArray(axis)) return Number(axis[1]) || 0;
    if (axis && typeof axis === 'object') {
      return Number(axis.y ?? axis.vertical ?? axis[1]) || 0;
    }
    return Number(axis) || 0;
  }

  function newOperationId() {
    const cryptoApi = globalThis.crypto;
    if (cryptoApi && typeof cryptoApi.randomUUID === 'function') {
      return cryptoApi.randomUUID();
    }
    state.fallbackOperationId += 1;
    return 'reset-' + Date.now() + '-' + state.fallbackOperationId;
  }

  function showFailure(error) {
    const detail = error && error.message ? ': ' + error.message : '';
    if (typeof globalThis.navShowNotice === 'function') {
      globalThis.navShowNotice('复位请求失败，请松开按键后重试' + detail, 3500, '#FF8A8A');
    }
  }

  function settleFailure(error) {
    state.busy = false;
    const released = areReleased(state.previous);
    state.requestInputsReleased = released;
    state.awaitingPostRequestRelease = !released;
    state.confirmAArmed = released && state.open && state.page === 'confirm';
    showFailure(error);
    redraw();
  }

  function finishClose() {
    state.execution = null;
    state.waitingRun = false;
    state.stopWanted = state.stopPending = false;
    state.stopArmed = {a:false, b:false};
    state.closeAt = 0;
    state.statusMessage = '';
    state.requestToken += 1;
    state.busy = false;
    state.open = false;
    state.page = 'menu';
    state.selected = 0;
    state.ybCandidate = false;
    state.ybStartedAt = null;
    state.xStartedAt = null;
    state.axisLatched = false;
    state.confirmAArmed = false;
    state.awaitingPostRequestRelease = false;
    state.requestInputsReleased = false;
    state.consumeTail = true;
    state.ignoreBackUntilRelease = true;
    redraw();
  }

  async function resetAction(endpoint, payload) {
    // Stop must bypass navigationAction's shared actionPending mutex.
    if (typeof globalThis.fetch !== 'function') return globalThis.navigationAction(endpoint, payload);
    const abort = new AbortController();
    const timeout = globalThis.setTimeout(() => abort.abort(), 6500);
    try {
      const response = await globalThis.fetch(endpoint, {
        method:'POST', headers:{'Content-Type':'application/json'},
        body:JSON.stringify(payload), signal:abort.signal
      });
      const result = await response.json();
      if (!response.ok && result.success !== false) throw new Error('HTTP '+response.status);
      return result;
    } finally {globalThis.clearTimeout(timeout);}
  }

  function requestReset() {
    state.baselineId = state.lastRun?.id || null;
    state.waitingRun = true;
    state.page = 'running';
    state.statusMessage = '正在提交复位请求';
    state.stopArmed = {a:false, b:false};
    state.busy = true;
    state.confirmAArmed = false;
    state.awaitingPostRequestRelease = true;
    state.requestInputsReleased = false;
    const token = ++state.requestToken;
    redraw();

    let result;
    try {
      const payload = {
        client_id: globalThis.autolifeVrClientId?.(),
        operation_id: newOperationId(),
        scope: SCOPES[state.selected],
        confirmed: true,
        preserve_grippers: true
      };
      if (typeof globalThis.navigationAction !== 'function') {
        throw new Error('导航请求接口不可用');
      }
      result = resetAction('/api/reset/arms', payload);
    } catch (error) {
      state.waitingRun = false;
      state.page = 'menu';
      settleFailure(error);
      return;
    }

    Promise.resolve(result).then((response) => {
      if (token !== state.requestToken) return;
      if (response === true || (response && response.success === true)) {
        // Acceptance is not completion. Status owns the modal until terminal.
        if (!state.execution) state.statusMessage = '等待机器人复位状态';
        redraw();
        return;
      }
      if (!state.execution) {
        state.waitingRun = false;
        state.page = 'menu';
        settleFailure(new Error(response?.message || '复位请求被拒绝'));
      }
    }, (error) => {
      if (token !== state.requestToken) return;
      // A lost HTTP reply does not prove the robot rejected the request.
      state.statusMessage = '请求结果未确认，正在核对机器人状态';
      redraw();
    });
  }


  function terminalRun(run) {
    return ['completed', 'stopped', 'failed'].includes(run?.state);
  }

  function acceptRun(run) {
    state.open = true;
    state.busy = true;
    state.page = 'running';
    state.waitingRun = false;
    state.execution = {...run};
    if (terminalRun(run)) {
      state.stopPending = false;
      state.stopWanted = false;
      if (!state.closeAt && run.state !== 'failed') state.closeAt = Date.now() + 1200;
    }
    redraw();
  }

  function updateStatus(teleop, options = {}) {
    if (options.fresh !== true) {
      if (state.execution || state.waitingRun) {
        state.statusMessage = '连接或反馈中断，停止状态尚未确认';
        state.closeAt = 0;
        redraw();
      }
      return;
    }
    state.lastFreshAt = Date.now();
    const run = teleop?.backend?.reset_run;
    if (!run?.id) {
      if (state.waitingRun) state.statusMessage = '等待本次复位编号';
      return;
    }
    state.lastRun = {...run};
    const following = state.execution?.id === run.id;
    const awaited = state.waitingRun && run.id !== state.baselineId;
    const active = run.state === 'running' && options.xrActive === true;
    if (following || awaited || active) {
      if (!following) {
        if (state.execution) state.stopWanted = state.stopPending = false;
        state.stopArmed = {a:false, b:false};
        state.closeAt = 0;
      }
      state.statusMessage = '';
      acceptRun(run);
      if (state.stopWanted && run.state === 'running') requestStop();
    }
  }

  function requestStop() {
    state.stopWanted = true;
    if (!state.execution?.id || state.stopPending) {
      state.statusMessage = state.execution ? '正在停止' : '正在获取复位编号，随后停止';
      redraw();
      return;
    }
    const id = state.execution.id;
    state.stopPending = true;
    state.statusMessage = '正在停止';
    redraw();
    Promise.resolve().then(() => resetAction('/api/reset/stop', {
      client_id: globalThis.autolifeVrClientId?.(), reset_id: id
    })).then(response => {
      if (state.execution?.id !== id) return;
      state.stopPending = false;
      if (response?.success && response.reset?.id === id && terminalRun(response.reset)) {
        state.lastFreshAt = Date.now();
        state.statusMessage = '';
        acceptRun(response.reset);
      } else {
        state.stopWanted = false;
        state.statusMessage = response?.message || '停止尚未确认，请再按 A 或 B';
        redraw();
      }
    }, () => {
      if (state.execution?.id !== id) return;
      state.stopPending = state.stopWanted = false;
      state.statusMessage = '停止尚未确认，请检查连接后再按 A 或 B';
      redraw();
    });
  }

  function inputLost() {
    if ((state.execution && !terminalRun(state.execution)) || state.waitingRun) {
      requestStop();
    } else if (!state.execution) finishClose();
  }

  function handleExecution(current, previous) {
    if (!state.execution && !state.waitingRun) return false;
    // Arm each key independently: a held reset-start A must not cancel the
    // reset, and must not prevent B from stopping it (or vice versa).
    for (const key of ['a', 'b']) {
      if (!current[key]) state.stopArmed[key] = true;
    }
    const stale = Date.now() - state.lastFreshAt > 2000;
    if (stale) state.closeAt = 0;
    if (state.closeAt && Date.now() >= state.closeAt && !stale) {
      finishClose();
      setPrevious(current);
      return true;
    }
    const stopPressed = ['a', 'b'].some(key =>
      current[key] && !previous[key] && state.stopArmed[key]);
    if (stopPressed) {
      if (state.execution?.state === 'failed' && !stale &&
          current.b && !previous.b && state.stopArmed.b) finishClose();
      else if (!terminalRun(state.execution)) requestStop();
    }
    setPrevious(current);
    return true;
  }

  function handle(input = {}) {
    const current = normalizeButtons(input);
    const previous = state.previous;
    if (handleExecution(current, previous)) return true;
    const allReleased = areReleased(current);
    const nowValue = Number(input.now);
    const now = Number.isFinite(nowValue) ? nowValue : Date.now();
    const ybHeld = current.y && current.b;
    let consumedThisFrame = false;

    if (ybHeld) {
      state.ybCandidate = true;
      state.consumeTail = true;
      if (state.ybStartedAt === null) state.ybStartedAt = now;
      if (!state.open && !state.busy && now - state.ybStartedAt >= HOLD_MS) {
        state.open = true;
        state.page = 'menu';
        state.selected = 0;
        state.confirmAArmed = false;
        state.awaitingPostRequestRelease = false;
        state.ignoreBackUntilRelease = true;
        state.axisLatched = Math.abs(getAxisY(input.axis)) >= AXIS_ON;
        redraw();
      } else if (state.open) {
        state.ignoreBackUntilRelease = true;
      }
    } else if (state.ybStartedAt !== null) {
      state.ybStartedAt = null;
    }

    if (state.open && state.page === 'confirm' && !state.busy && !state.awaitingPostRequestRelease && !current.a && previous.a) {
      state.confirmAArmed = true;
    }

    if (allReleased) {
      if (state.ignoreBackUntilRelease || state.consumeTail || state.ybCandidate) {
        consumedThisFrame = true;
      }
      state.ignoreBackUntilRelease = false;
      state.consumeTail = false;
      state.ybCandidate = false;
      if (state.awaitingPostRequestRelease) {
        state.requestInputsReleased = true;
        if (!state.busy) {
          state.awaitingPostRequestRelease = false;
          if (state.open && state.page === 'confirm') state.confirmAArmed = true;
        }
      }
    }

    if (state.busy) {
      setPrevious(current);
      return true;
    }

    if (state.open) {
      if (state.ignoreBackUntilRelease) {
        setPrevious(current);
        return true;
      }

      if (current.x && !previous.x) state.xStartedAt = now;
      if (current.x && state.xStartedAt !== null && now - state.xStartedAt >= HOLD_MS) {
        finishClose();
        setPrevious(current);
        return true;
      }
      if (!current.x && previous.x && state.xStartedAt !== null) {
        const heldFor = now - state.xStartedAt;
        state.xStartedAt = null;
        if (heldFor >= HOLD_MS) {
          finishClose();
          if (allReleased) {
            state.consumeTail = false;
            state.ignoreBackUntilRelease = false;
          }
        }
        setPrevious(current);
        return true;
      }
      if (current.x) {
        setPrevious(current);
        return true;
      }

      if (state.page === 'menu') {
        const axisY = getAxisY(input.axis);
        if (Math.abs(axisY) <= AXIS_OFF) state.axisLatched = false;
        if (!state.axisLatched && Math.abs(axisY) >= AXIS_ON) {
          state.axisLatched = true;
          const next = Math.max(0, Math.min(SCOPES.length - 1, state.selected + (axisY > 0 ? 1 : -1)));
          if (next !== state.selected) {
            state.selected = next;
            redraw();
          }
        }
        if (current.b && !previous.b && !current.a && !current.y) {
          finishClose();
        } else if (current.a && !previous.a && !current.b && !current.y &&
          !state.awaitingPostRequestRelease) {
          // The selected row is the confirmation. One A submits the scoped
          // reset; the backend still validates hardware and follow state.
          requestReset();
        }
      } else {
        if (current.b && !previous.b) {
          state.page = 'menu';
          state.confirmAArmed = false;
          redraw();
        } else if (current.a && !previous.a && !current.b && !current.y && state.confirmAArmed && !state.awaitingPostRequestRelease) {
          requestReset();
        }
      }

      setPrevious(current);
      return true;
    }

    const owned = ybHeld || state.ybCandidate || state.ybStartedAt !== null || state.consumeTail || consumedThisFrame;
    setPrevious(current);
    return Boolean(owned);
  }

  function ownsInput() {
    return state.open || state.busy || state.ybCandidate || state.ybStartedAt !== null || state.consumeTail || state.ignoreBackUntilRelease;
  }

  function buttonValue(controller, key) {
    return Number(controller && controller[key]) || 0;
  }

  function hasYBChord(packet) {
    const left = packet && packet.leftController;
    const right = packet && packet.rightController;
    const controllers = packet && packet.controllers;
    const leftY = buttonValue(left, 'yButton') || buttonValue(controllers && controllers.left, 'yButton') || buttonValue(packet, 'yButton');
    const rightB = buttonValue(right, 'bButton') || buttonValue(controllers && controllers.right, 'bButton') || buttonValue(packet, 'bButton');
    return leftY > 0 && rightB > 0;
  }

  function gripsReleased(left, right) {
    if (!left || !right) return false;
    return !left.gripActive && Number(left.gripValue) < 0.15 &&
      !right.gripActive && Number(right.gripValue) < 0.15;
  }

  function clearControllerInput(controller) {
    if (!controller || typeof controller !== 'object') return;
    controller.xButton = 0;
    controller.yButton = 0;
    controller.aButton = 0;
    controller.bButton = 0;
    controller.gripActive = false;
    controller.grip = false;
    controller.gripValue = 0;
    controller.trigger = 0;
    controller.thumbstick = { x: 0, y: 0, pressed: 0 };
  }

  function filterPacket(packet) {
    if (!packet || typeof packet !== 'object') return packet;

    // The first combined packet establishes ownership; handle() owns the hold timer.
    if (hasYBChord(packet)) {
      state.ybCandidate = true;
      state.consumeTail = true;
    }

    if (!ownsInput()) return packet;

    const left = packet.leftController || (packet.controllers && packet.controllers.left);
    const right = packet.rightController || (packet.controllers && packet.controllers.right);
    packet.resetMenuGripsReleased = gripsReleased(left, right);

    clearControllerInput(packet.leftController);
    clearControllerInput(packet.rightController);
    if (packet.controllers) {
      clearControllerInput(packet.controllers.left);
      clearControllerInput(packet.controllers.right);
    }

    const rootKeys = ['xButton', 'yButton', 'aButton', 'bButton', 'gripActive', 'grip', 'gripValue', 'trigger', 'thumbstick'];
    for (const key of rootKeys) {
      if (Object.prototype.hasOwnProperty.call(packet, key)) {
        packet[key] = key === 'thumbstick' ? { x: 0, y: 0, pressed: 0 } : (key.endsWith('Button') || key === 'trigger' || key === 'gripValue' ? 0 : false);
      }
    }
    packet.resetMenuActive = true;
    return packet;
  }

  function roundedRect(context, x, y, width, height, radius) {
    if (typeof context.roundRect === 'function') {
      context.beginPath();
      context.roundRect(x, y, width, height, radius);
      return;
    }
    context.beginPath();
    context.moveTo(x + radius, y);
    context.lineTo(x + width - radius, y);
    context.quadraticCurveTo(x + width, y, x + width, y + radius);
    context.lineTo(x + width, y + height - radius);
    context.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
    context.lineTo(x + radius, y + height);
    context.quadraticCurveTo(x, y + height, x, y + height - radius);
    context.lineTo(x, y + radius);
    context.quadraticCurveTo(x, y, x + radius, y);
    context.closePath();
  }

  function drawExecution(context) {
    const run = state.execution;
    const width = context.canvas?.width || 1200;
    const height = context.canvas?.height || 900;
    const w = Math.min(680, width - 48), h = 310;
    const x = (width-w)/2, y = (height-h)/2;
    const outcome = run?.state;
    const titles = {completed:'复位完成', stopped:'复位已停止', failed:'复位异常'};
    const title = titles[outcome] || (state.stopWanted ? '正在停止复位' : '正在复位');
    const progress = outcome === 'completed' ? 100 : Math.min(99, Math.max(0, Math.floor((run?.progress || 0)*100)));
    const stale = Date.now() - state.lastFreshAt > 2000;
    const accent = outcome === 'failed' || stale ? '#FF8A8A' : '#72D8C3';
    context.save?.();
    roundedRect(context,x,y,w,h,12); context.fillStyle='#17242D'; context.fill?.();
    context.strokeStyle=accent; context.lineWidth=2; context.stroke?.();
    context.textAlign='center'; context.textBaseline='middle';
    context.fillStyle='#F5FAFC'; context.font='bold 32px sans-serif';
    context.fillText?.(title,width/2,y+48);
    context.font='20px sans-serif'; context.fillStyle='#BDD0D7';
    const scope = {left:'左臂',right:'右臂',both:'双臂',head:'头部',system:'系统'}[run?.scope] || '机器人';
    context.fillText?.(scope+'复位 · '+progress+'%',width/2,y+100);
    roundedRect(context,x+48,y+133,w-96,14,7);context.fillStyle='#30444F';context.fill?.();
    if(progress>0) {roundedRect(context,x+48,y+133,(w-96)*progress/100,14,7);context.fillStyle=accent;context.fill?.();}
    context.font='19px sans-serif';context.fillStyle=accent;
    const note = stale ? '反馈中断，停止状态尚未确认' : (state.statusMessage ||
      (terminalRun(run) ? (outcome==='failed' ? '请检查机器人 · 按 B 返回' : '即将返回原界面') : '其他操作已屏蔽'));
    context.fillText?.(note,width/2,y+187,w-64);
    roundedRect(context,x+120,y+223,w-240,56,9);
    context.fillStyle=terminalRun(run)?'#245B58':'#933E46';context.fill?.();
    context.font='bold 23px sans-serif';context.fillStyle='#F5FAFC';
    context.fillText?.(terminalRun(run)?(outcome==='failed'?'B 返回':'松开再按腰键接管'):'A 或 B  停止复位',width/2,y+251);
    context.restore?.();
  }

  function draw(context) {
    if (!state.open || !context) return;
    if (state.execution || state.waitingRun) {drawExecution(context); return;}
    const width = Number(context.canvas && context.canvas.width) || 1200;
    const height = Number(context.canvas && context.canvas.height) || 900;
    const panelWidth = Math.min(760, width - 48);
    const panelHeight = state.page === 'menu' ? 500 : 430;
    const left = (width - panelWidth) / 2;
    const top = (height - panelHeight) / 2;

    context.save?.();
    // Keep the live camera view visible around the floating panel.
    roundedRect(context, left, top, panelWidth, panelHeight, 12);
    context.fillStyle = '#17242D';
    context.fill?.();
    context.lineWidth = 2;
    context.strokeStyle = '#72D8C3';
    context.stroke?.();

    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillStyle = '#F5FAFC';
    context.font = 'bold 34px sans-serif';
    context.fillText?.(state.page === 'menu' ? '机械臂复位' : '确认复位', width / 2, top + 56);

    context.fillStyle = '#BDD0D7';
    context.font = '21px sans-serif';
    if (state.page === 'menu') {
      context.fillText?.('选择复位范围 · 保留左右夹爪状态', width / 2, top + 105);
      for (let index = 0; index < LABELS.length; index += 1) {
        const rowY = top + 158 + index * 76;
        const rowX = left + 74;
        const rowWidth = panelWidth - 148;
        roundedRect(context, rowX, rowY, rowWidth, 58, 8);
        context.fillStyle = index === state.selected ? '#245B58' : '#24333D';
        context.fill?.();
        context.strokeStyle = index === state.selected ? '#72D8C3' : '#40545E';
        context.lineWidth = index === state.selected ? 2 : 1;
        context.stroke?.();
        context.fillStyle = '#F5FAFC';
        context.font = index === state.selected ? 'bold 25px sans-serif' : '23px sans-serif';
        context.fillText?.(LABELS[index], width / 2, rowY + 29);
      }
      context.fillStyle = '#C9D7DC';
      context.font = '18px sans-serif';
      context.fillText?.('右摇杆 ↑↓ 选择　　A 直接复位　　B 取消　　长按 X 1 秒退出', width / 2, top + panelHeight - 40);
    } else {
      context.fillText?.('复位范围：' + LABELS[state.selected], width / 2, top + 126);
      context.fillStyle = '#72D8C3';
      context.font = 'bold 24px sans-serif';
      context.fillText?.('左右夹爪状态都会保留', width / 2, top + 190);
      context.fillStyle = '#F0C979';
      context.font = '20px sans-serif';
      context.fillText?.(state.busy ? '正在提交复位请求' : '请确认周围安全后继续', width / 2, top + 246);
      context.fillStyle = '#C9D7DC';
      context.font = '18px sans-serif';
      context.fillText?.(state.busy ? '请求处理中，按键已锁定' : 'A 确认　　B 返回　　长按 X 1 秒退出', width / 2, top + panelHeight - 48);
    }
    context.restore?.();
  }

  return {
    handle,
    ownsInput,
    filterPacket,
    draw,
    updateStatus,
    inputLost,
    isOpen: () => state.open,
    close: inputLost
  };
})();

globalThis.resetControls = resetControls;
globalThis.autolifeResetControls = resetControls;
if (typeof module !== 'undefined' && module.exports) module.exports = resetControls;
