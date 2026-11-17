/* Explicit squat choice. No mode selection sends a height or joint target. */
class SquatMenuGate {
  constructor() { this.clear(); }
  clear() { this.open=false; this.awaitNeutral=false; }
  show() { this.open=true; this.awaitNeutral=true; }
  close() { this.open=false; this.awaitNeutral=true; }
  get paused() { return this.open || this.awaitNeutral; }
  releaseIfNeutral(buttons, axes) {
    if (!this.open && !buttons.some(Boolean)
        && axes.every(v=>Number.isFinite(v) && Math.abs(v)<.15)) this.awaitNeutral=false;
  }
}

globalThis.squatModeUi = {
  gate: new SquatMenuGate(), selected:0, axisLatch:false, pending:false,
  receivedAt:-Infinity, generation:0,
  get paused() { return this.gate.paused || this.pending; },
  reset() { this.generation++; this.pending=false; this.gate.clear(); },
  status() {
    const s=navUi.state?.manual_squat || {};
    if(performance.now()-this.receivedAt>1800) return {mode:null,can_switch:false,message:'等待腰腿反馈'};
    return s;
  },
  show() {
    if(this.status().automatic_height_active) {
      navShowNotice('请等待自动身体高度调整结束，再选择下蹲方式',2600,'#ffcf70');return;
    }
    this.gate.show(); this.selected=this.status().mode==='reverse'?1:0;
    this.axisLatch=true; navUi.bodyHeightEngaged=false;
    sendBaseDrive(true,0,0,0,false,0,true);
    navShowNotice('底盘与升降已暂停\n摇杆选择，A 确认；B 返回',2200,'#68e1ff');
    drawNavigationPanel();
  },
  close() {
    this.gate.close();
    navShowNotice('请松开按键、摇杆回中\n随后恢复底盘控制',2200,'#68e1ff');
    drawNavigationPanel();
  },
  async choose(right) {
    if(this.pending) return;
    const status=this.status();
    if(!status.can_switch) {navShowNotice(status.message || '暂不能切换',2600,'#ffcf70');return;}
    const generation=this.generation;
    this.pending=true; drawNavigationPanel();
    try {
      const response=await fetch('/api/body-height/mode', {method:'POST',
        headers:{'Content-Type':'application/json'}, body:JSON.stringify({
          mode:this.selected===1?'reverse':'normal', client_id:globalThis.autolifeVrClientId?.() || ''})});
      const result=await response.json();
      if(generation!==this.generation) return;
      if(!response.ok || !result.success) throw new Error(result.message || '切换未确认');
      navUi.state.manual_squat=result.manual_squat; this.receivedAt=performance.now();
      this.gate.close();
      navShowNotice(`${result.message}\nX 降低 / Y 升高；松键并回中后继续`,2800,'#68e1ff');
      try { const actuator=right?.hapticActuators?.[0] || right?.vibrationActuator;
        await actuator?.pulse?.(.25,80); } catch (_) { /* Haptics are optional. */ }
    } catch(error) {
      if(generation===this.generation) navShowNotice(error.message,3000,'#ffcf70');
    } finally {
      if(generation===this.generation) {this.pending=false;drawNavigationPanel();}
    }
  },
  handleInput(left,right,keys) {
    if(!navUi.baseControlLatched || !this.paused) return false;
    const la=left.axes || [], ra=right.axes || [];
    const ls=Boolean(left.buttons?.[3]?.pressed || (!left.buttons?.[3] && left.buttons?.[2]?.pressed));
    const rs=Boolean(right.buttons?.[3]?.pressed || (!right.buttons?.[3] && right.buttons?.[2]?.pressed));
    sendBaseDrive(true,0,0,0,false,0,true);navUi.bodyHeightEngaged=false;
    // Existing reset chords retain priority. Their ordinary handlers receive
    // the same input; the menu continues to suppress base/height commands.
    if((keys.x&&keys.a)||(keys.y&&keys.b)||navUi.holdGripperResetChord) {
      this.gate.close();return false;
    }
    if(ls && (!navUi.previous.leftStick || rs)) {
      setBaseControlLatched(false,!rs,!rs);return true;
    }
    const menu=globalThis.autolifeTeaching
      ? autolifeTeaching.menuPressed(rs&&!navUi.previous.rightStick)
      : rs&&!navUi.previous.rightStick;
    if(this.gate.open) {
      if(!this.pending && (menu || (keys.b&&!navUi.previous.b))) this.close();
      else if(!this.pending) {
        const axis=ra[3] || 0;
        if(Math.abs(axis)<.35) this.axisLatch=false;
        if(!this.axisLatch && Math.abs(axis)>.68) {
          this.selected=axis>0?1:0;this.axisLatch=true;drawNavigationPanel();
        }
        if(keys.a&&!navUi.previous.a&&!keys.x) void this.choose(right);
      }
    } else if(!this.pending) {
      this.gate.releaseIfNeutral([...Object.values(keys),ls,rs],
        [la[2]||0,la[3]||0,ra[2]||0,ra[3]||0]);
    }
    navUi.previous={...keys,leftStick:ls,rightStick:rs};
    return true;
  },
  draw(ctx) {
    const s=this.status(), label=s.mode==='reverse'?'反向下蹲':s.mode==='normal'?'正向下蹲':'下蹲方式待同步';
    ctx.save();ctx.textAlign='left';
    roundRect(ctx,570,18,220,48,12,'rgba(4, 18, 27, 0.94)');
    ctx.fillStyle='#eafaff';ctx.font='bold 21px sans-serif';ctx.fillText(label,585,49);
    if(navUi.baseControlLatched) {
      roundRect(ctx,350,808,500,40,12,'rgba(4, 18, 27, 0.94)');
      ctx.fillStyle='#b8d9e4';ctx.font='18px sans-serif';
      ctx.fillText(this.paused?'底盘 / 升降暂停 · 松键并回中后继续':'X 降低 / Y 升高 · 短按右摇杆选下蹲方式',367,834);
    }
    if(this.gate.open) {
      roundRect(ctx,340,180,660,440,22,'rgba(3, 15, 25, 0.98)');
      ctx.fillStyle='#eafaff';ctx.font='bold 28px sans-serif';ctx.fillText('选择下蹲方式',365,225);
      ctx.font='18px sans-serif';ctx.fillStyle='#b8d9e4';ctx.fillText('仅切换方式，不改变当前身体高度',365,257);
      ['正向下蹲','反向下蹲'].forEach((text,i)=>{
        const y=280+i*112, chosen=i===this.selected;
        roundRect(ctx,365,y,610,100,14,chosen?'#185066':'#152a37');
        ctx.strokeStyle=chosen?'#68e1ff':'#728b97';ctx.lineWidth=5;ctx.beginPath();
        // Side-view pictogram of the two bending directions (not live FK).
        const x=420,d=i===0?1:-1;
        ctx.moveTo(x,y+15);ctx.lineTo(x,y+32);ctx.lineTo(x+d*23,y+54);
        ctx.lineTo(x,y+78);ctx.lineTo(x+22,y+78);ctx.stroke();
        ctx.fillStyle='#eafaff';ctx.font='bold 25px sans-serif';ctx.fillText(text,485,y+42);
        const actual=(i===0?'normal':'reverse')===s.mode;
        ctx.fillStyle='#b8d9e4';ctx.font='18px sans-serif';ctx.fillText(actual?'当前方式':'直立停稳后可切换',485,y+73);
      });
      ctx.font='18px sans-serif';ctx.fillStyle=s.can_switch?'#68e1ff':'#ffcf70';
      ctx.fillText(this.pending?'等待控制器确认…':(s.message||'等待状态'),365,542);
      ctx.fillStyle='#eafaff';ctx.fillText('右摇杆上下选择 · A 确认 · B / 短按右摇杆返回',365,580);
    }
    ctx.restore();
  }
};
if(typeof module!=='undefined') module.exports={SquatMenuGate,ui:globalThis.squatModeUi};
