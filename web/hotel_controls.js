/* Hotel evidence controls use existing menu navigation, A confirmation and B head follow. */
const hotelControls = {
  open: false, catalog: null, pending: null, busy: false,
  disposalToken: '', modalPending: null, modalReady: false, modalTail: false,
  modalPrevious: {a:false,b:false,x:false,y:false}, modalXAt: null, modalAxisLatched: false,
  mission() { return navUi.state?.hotel_mission || {}; },
  payload(extra = {}) {
    return {client_id: globalThis.autolifeVrClientId?.(), token: this.mission().token,
      operation_id: globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random()}`, ...extra};
  },
  items() {
    if (!this.open) return null;
    const row = (action, label, extra = {}) => ({kind: 'hotel-action', action, label, ...extra});
    if (this.pending) return [
      {kind: 'hotel-back', label: '← 返回，暂不确认'},
      {kind: 'hotel-confirm', label: this.pending.label}
    ];
    const m = this.mission(), s = m.state;
    const items = [{kind: 'hotel-close', label: '← 返回主菜单'}];
    if (!m.active) {
      return items.concat((this.catalog?.missions || []).map(x =>
        ({kind: 'hotel-start', id: x.id, label: `开始：${x.name || x.id}`})));
    }
    items.push({kind:'hotel-abort',label:'中止本次巡查…（需确认）'});
    if (s === 'PICKING_BASKET') items.push(row('pickup_failed', '前台取篮失败（记录）', {confirm: true}));
    if (s === 'PICKUP_FAILED') items.push(row('retry_pickup', '原地重试取篮', {confirm: true}));
    if (['WAITING_FOR_CLEANING', 'WAITING_AREA_CONFIRM'].includes(s)) {
      items.push(row('photo', '补拍当前桌面'), row('basket_full', '篮子已满（记录）'),
        row('complete', '当前点位：操作失败', {result: 'failed', confirm: true}),
        row('complete', '当前点位：没有目标物', {result: 'no_target', confirm: true}));
      for (const [label, name] of [['water_bottle', '水瓶'], ['water_cup', '水杯'],
        ['tissue', '纸巾'], ['paper', '纸张'], ['other', '其他']])
        items.push(row('mark_item', `记一件清理物：${name}`, {item_label: label}));
    }
    if (['PICKING_BASKET','WAITING_FOR_CLEANING','WAITING_AREA_CONFIRM','DUMPING'].includes(s)
        && ['adjusting_height','failed'].includes(navUi.state?.task_dispatch?.state))
      items.push(row('manual_ready', '人工确认当前姿态可作业', {confirmed: true, confirm: true}));
    const retryingDumpAtBin = s === 'PAUSED_FOR_DISPOSAL' && m.last_failure?.code === 'DUMP_FAILED';
    if (!retryingDumpAtBin && ((m.basket_state === 'full' && ['WAITING_FOR_CLEANING','WAITING_AREA_CONFIRM','PAUSED_FOR_DISPOSAL'].includes(s))
        || (s === 'PAUSED_FOR_DISPOSAL' && m.pending_disposal_kind === 'scheduled_final')))
      items.push(row('dispose', '收臂就绪，前往垃圾桶', {arms_secured: true, confirm: true}));
    if (s === 'DUMPING') items.push(row('complete', '垃圾桶作业成功', {confirm: true}),
      row('dump_failed', '垃圾桶作业失败（记录）', {confirm: true}));
    if (s === 'PAUSED_FOR_DISPOSAL' && m.last_failure?.code === 'DUMP_FAILED')
      items.push(row('retry_dump', '原地重试倒垃圾', {confirm: true}));
    if (s === 'PAUSED') items.push(row('retry', '重试当前导航', {confirm: true}));
    if (m.current_area_id && ['NAVIGATING_TO_AREA','WAITING_FOR_CLEANING','WAITING_AREA_CONFIRM','PAUSED'].includes(s))
      items.push(row('skip_area', '跳过当前点位并记录失败', {confirmed: true, confirm: true}));
    if (['WAITING_FOR_CLEANING','WAITING_AREA_CONFIRM'].includes(s)) {
      for (const [code,label] of [['GRASP_FAILED','夹取失败'],['OBJECT_DROPPED','物品掉落'],
        ['SCENE_CHANGED','桌面环境已改变'],['VIEW_BLOCKED','视野被遮挡']])
        items.push(row('exception', `记录：${label}`, {code}));
    }
    return items;
  },
  syncDisposal() {
    const m = this.mission();
    const waiting = m.active && m.state === 'PAUSED_FOR_DISPOSAL' && m.last_failure?.code !== 'DUMP_FAILED';
    if (this.pending?.path === '/api/hotel/action' && this.pending.payload.token !== m.token) {
      this.pending = null;
      this.modalTail = true;
    }
    if (!waiting) { this.disposalToken = ''; return false; }
    if (this.disposalToken === m.token) return false;
    this.disposalToken = m.token;
    this.open = true;
    this.pending = {label:'确认手臂和篮子已收妥，前往垃圾桶',path:'/api/hotel/action',
      payload:this.payload({action:'dispose',arms_secured:true})};
    navUi.menuVisible = true;
    navUi.selected = 0;
    drawNavigationPanel();
    return true;
  },
  ownsConfirmation() { return Boolean(this.pending || this.modalTail); },
  filterPacket(packet) {
    if (!this.ownsConfirmation() || !packet || typeof packet !== 'object') return packet;
    for (const hand of ['leftController','rightController']) {
      if (!packet[hand]) continue;
      packet[hand] = {...packet[hand],xButton:0,yButton:0,aButton:0,bButton:0,
        thumbstick:{x:0,y:0,pressed:0}};
    }
    return packet;
  },
  handleConfirmation({a,b,x,y,axis,now}) {
    const buttons = {a:Boolean(a),b:Boolean(b),x:Boolean(x),y:Boolean(y)};
    const released = !a && !b && !x && !y;
    if (!this.pending) {
      this.modalPending = null;
      if (!this.modalTail) return false;
      if (released) this.modalTail = false;
      this.modalPrevious = buttons;
      return true;
    }
    if (this.modalPending !== this.pending) {
      this.modalPending = this.pending;
      this.modalReady = false;
      this.modalXAt = null;
      this.modalAxisLatched = Math.abs(axis) >= .68;
      this.modalPrevious = buttons;
    }
    this.modalTail = true;
    if (this.busy) { this.modalPrevious = buttons; return true; }
    if (!this.modalReady) {
      if (released) this.modalReady = true;
      this.modalPrevious = buttons;
      return true;
    }
    if (x && !a && !b && !y) {
      if (this.modalXAt === null) this.modalXAt = now;
      if (now - this.modalXAt >= 1000) {
        this.pending = null;
        this.open = false;
        navUi.menuVisible = false;
        navUi.selected = 0;
        drawNavigationPanel();
        navShowNotice(this.mission().active ? '已返回，巡查进度保留；从“当前巡查操作”可重新确认'
          : '已关闭确认提示；需要时可从菜单重新选择',4000,'#ffcf70');
      }
      this.modalPrevious = buttons;
      return true;
    }
    this.modalXAt = null;
    if (b && !this.modalPrevious.b && !a && !x && !y) {
      this.pending = null;
      navUi.selected = 0;
      drawNavigationPanel();
    } else if (!a && !b && !x && !y) {
      if (Math.abs(axis) < .35) this.modalAxisLatched = false;
      if (!this.modalAxisLatched && Math.abs(axis) > .68) {
        navUi.selected = axis > 0 ? 1 : 0;
        this.modalAxisLatched = true;
        drawNavigationPanel();
      }
    } else if (a && !this.modalPrevious.a && !b && !x && !y) {
      void this.activate({kind:navUi.selected === 1 ? 'hotel-confirm' : 'hotel-back'});
    }
    this.modalPrevious = buttons;
    return true;
  },
  resetInput() {
    this.pending = null; this.open = false; this.modalPending = null; this.modalTail = false;
    this.modalReady = false; this.modalXAt = null; this.disposalToken = '';
  },
  async startFromChain(name) {
    await this.activate({kind: 'hotel-open'});
    const mission = this.catalog?.missions?.find(x => x.source_chain === name);
    if (!mission) { navShowNotice('任务配置已改变，请重新打开酒店巡查菜单', 4000, '#ffcf70'); return; }
    await this.activate({kind: 'hotel-start', id: mission.id});
    navUi.menuVisible = true;
    navUi.selected = 1;
    drawNavigationPanel();
  },
  async activate(item) {
    if (!item.kind.startsWith('hotel-') || this.busy) return;
    this.busy = true;
    try {
      if (item.kind === 'hotel-open') {
        const response = await fetch('/api/hotel/catalog', {cache: 'no-store'});
        const data = await response.json();
        if (!response.ok || data.success === false) throw Error(data.message || '读取酒店任务配置失败');
        this.catalog = data; this.open = true; this.pending = null;
      } else if (item.kind === 'hotel-close') { this.open = false; this.pending = null;
      } else if (item.kind === 'hotel-back') { this.pending = null;
      } else if (item.kind === 'hotel-start') {
        this.pending = {label: '确认开始当前酒店任务串', path: '/api/hotel/start',
          payload: this.payload({mission_id: item.id, catalog_token: this.catalog.token,
            confirmed: true, start_at_dock_confirmed: true})};
      } else if (item.kind === 'hotel-abort') {
        this.pending = {label:'确认中止本次巡查并停止导航',path:'/api/tasks/cancel',
          payload:this.payload({confirmed:true})};
      } else if (item.kind === 'hotel-confirm') {
        const pending = this.pending;
        if (pending && await navigationAction(pending.path, pending.payload)) {
          this.pending = null;
          this.open = false; navUi.menuVisible = false;
        } else {
          this.modalReady = false;
        }
      } else {
        const {kind, label, confirm, ...extra} = item;
        const payload = this.payload(extra);
        if (extra.action === 'mark_item') {
          payload.label = extra.item_label;
          delete payload.item_label;
        }
        if (confirm) this.pending = {label, path: '/api/hotel/action', payload};
        else await navigationAction('/api/hotel/action', payload);
      }
    } catch (error) { navShowNotice(error.message, 5000, '#ffcf70');
    } finally { this.busy = false; navUi.selected = 0; drawNavigationPanel(); }
  },
  draw(context) {
    const m = this.mission();
    if (!m.active) return;
    const labels = {NAVIGATING_TO_PICKUP:'前往前台', PICKING_BASKET:'等待操作者确认',
      PICKUP_FAILED:'取篮失败，从酒店菜单原地重试',
      NAVIGATING_TO_AREA:'前往当前点位', CAPTURING_BEFORE:'记录到达照片',
      WAITING_FOR_CLEANING:'等待操作者作业', WAITING_AREA_CONFIRM:'等待确认',
      PAUSED_FOR_DISPOSAL:'等待收臂确认 · 右摇杆键→当前巡查操作', NAVIGATING_TO_BIN:'前往垃圾桶',
      DUMPING:'完成垃圾桶作业后长按 A 确认；默认记录成功', PAUSED:'导航失败，长按 A 重试', RETURNING_TO_DOCK:'返回充电桩'};
    roundRect(context, 420, 220, 406, 63, 12, 'rgba(4,28,39,0.94)');
    context.fillStyle = m.basket_state === 'full' ? '#ffcf70' : '#72f0b1';
    context.font = 'bold 16px sans-serif';
    const total = m.mission?.metadata?.config?.areas?.length || 4;
    const current = m.mission?.areas?.[m.area_index];
    const progress = m.area_index < 0 ? '准备' : `${current?.waypoint || current?.name || '当前点位'} ${Math.min(total, m.area_index + 1)} / ${total}`;
    context.fillText(`酒店任务 · ${progress} · 篮子${m.basket_state === 'full' ? '已满' : '正常'}`, 435, 244, 376);
    context.fillStyle = '#d9f8ff';
    context.fillText(labels[m.state] || m.state, 435, 269, 376);
  },
  drawConfirmation(context) {
    if (!this.pending || !context) return;
    const width = context.canvas?.width || 1200, height = context.canvas?.height || 900;
    const left = (width-760)/2, top = (height-400)/2;
    context.save();
    context.fillStyle='rgba(0,0,0,0.55)';context.fillRect(0,0,width,height);
    roundRect(context,left,top,760,400,16,'rgba(4,28,39,0.98)');
    context.textAlign='center';context.fillStyle='#eefbff';context.font='bold 28px sans-serif';
    context.fillText(this.pending.payload.action === 'dispose' ? '前往垃圾桶前确认' : '巡查操作确认',width/2,top+55);
    context.font='21px sans-serif';context.fillStyle='#ffd276';
    context.fillText(this.pending.label,width/2,top+110,710);
    if (this.pending.payload.action === 'dispose') {
      context.font='18px sans-serif';context.fillStyle='#bed9e2';
      context.fillText('人工确认手臂和篮子已收妥；锁定头部、松开双手腰键',width/2,top+150,710);
    }
    for (let index=0;index<2;index++) {
      const rowY=top+188+index*62;
      roundRect(context,left+90,rowY,580,48,9,index===navUi.selected?'#246b78':'#193d49');
      context.fillStyle='#ffffff';context.font='bold 21px sans-serif';
      context.fillText(index===0?'返回，暂不确认':this.busy?'正在处理…':'确认执行',width/2,rowY+32);
    }
    context.font='18px sans-serif';context.fillStyle='#a9d9e6';
    context.fillText('右摇杆上下选择 · 短按 A 确认 · B 返回 · 长按 X 退出提示',width/2,top+365,720);
    context.restore();
  }
};
globalThis.autolifeHotelControls = hotelControls;
