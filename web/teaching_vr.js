/* Recording controls are separate from the real-time pose/control channel. */
globalThis.autolifeTeaching = {
  data:{available:false,enabled:false}, input:new TeachingInput(), owns:false, menu:false,
  busy:false, notice:'', noticeUntil:0, hud:null, canvas:null, texture:null,
  resetHeld:false, resetHeldAt:0,
  async fetchTimed(url,options={},milliseconds=1500) {
    const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),milliseconds);
    try{return await fetch(url,{...options,signal:controller.signal});}finally{clearTimeout(timer);}
  },
  accept(data) {
    if(data.available && data.service_id===this.data.service_id && data.revision<this.data.revision)return;
    const justSaved=data.phase==='saved' && (this.data.phase!=='saved' || data.saved_name!==this.data.saved_name);
    this.data=data;
    if(typeof vrCustom!=='undefined' && vrCustom.mode==='teaching' && data.enabled===false){vrCustom.mode='';navUi.selected=0;}
    if(justSaved){this.notify('轨迹已保存并同步到动作编排器',false);this.pulse(.5,120);}
  },
  async poll() {
    try {
      const active=Boolean(state.xrSession && !state.observerOnly);
      let url='/api/teaching/status',options={cache:'no-store'};
      if (active && this.data.available && this.data.enabled) {
        url='/api/teaching/command';
        options={...options,method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
          op:'heartbeat',session:this.data.session,
          ready:Boolean(state.controlRequested && state.teleop.status?.hardware_enabled && !hardwareControlIsTransitioning() && vrInputIsFresh()),
          recording_ready:Boolean(state.controlRequested && (vrInputIsFresh() || state.teleop.status?.quick_reset?.active) &&
            (state.teleop.status?.hardware_enabled || state.teleop.status?.quick_reset?.active)),
          resetting:Boolean(state.teleop.status?.quick_reset?.active),reset_held:this.resetHeld,
          reset_hold_age:this.resetHeld?(performance.now()-this.resetHeldAt)/1000:0
        })};
      }
      const response=await this.fetchTimed(url,options),data=await response.json();
      if (!response.ok || !data.success) throw Error(data.message||'示教服务断开');
      this.accept(data);
    } catch (error) {
      this.data={...this.data,available:false};this.notice='录制连接中断，重新连接后查看保存结果';
    }
    this.draw();
    if (typeof vrCustom!=='undefined' && vrCustom.mode==='teaching') drawNavigationPanel();
    setTimeout(()=>this.poll(),this.data.enabled?400:1000);
  },
  async command(op,values={}) {
    if (this.busy) return;
    this.busy=true;
    const id=globalThis.crypto?.randomUUID?.()||`${Date.now()}-${Math.random()}`;
    const body={op,session:this.data.session,request_id:id,...values};
    try {
      let data;
      for(let attempt=0;attempt<2;attempt++) {
        try {
          const response=await this.fetchTimed('/api/teaching/command',{method:'POST',
            headers:{'Content-Type':'application/json'},body:JSON.stringify(body)},2200);
          data=await response.json();
          if(!response.ok||!data.success)throw Object.assign(Error(data.message||'操作未完成'),{rejected:true});
          break;
        } catch(error) {if(attempt||error.rejected)throw error;}
      }
      this.accept(data);this.notify(data.message,false);this.pulse(.35,60);
    } catch(error) {this.notify(error.message||'未收到保存确认，请查看桌面状态',true);this.pulse(.8,180);}
    finally {this.busy=false;this.draw();if(typeof drawNavigationPanel==='function')drawNavigationPanel();}
  },
  notify(text,error) {
    this.notice=String(text);this.noticeUntil=performance.now()+4000;this.noticeError=error;
    if(typeof navShowNotice==='function')navShowNotice(text,3500,error?'#ff9b8a':'#7ee3bf');
  },
  pulse(value,duration) {
    for(const source of state.xrSession?.inputSources||[]) {
      if(source.handedness!=='right')continue;
      const actuator=source.gamepad?.hapticActuators?.[0];
      try {actuator?.pulse(value,duration)?.catch(()=>{});}catch(_){}
    }
  },
  tick(left,right,tracked) {
    const resetHeld=Boolean(left?.x && right?.a);
    if(resetHeld && !this.resetHeld)this.resetHeldAt=performance.now();
    this.resetHeld=resetHeld;
    const result=this.input.tick({enabled:Boolean(this.data.enabled),
      tracked:Boolean(tracked?.right),left:Boolean(left?.thumbstick?.pressed),right:Boolean(right?.thumbstick?.pressed)});
    this.owns=result.owns;
    for(const event of result.events) {
      if(event==='menu')this.menu=true;
      else if(this.data.available)void this.command(event);
      else this.notify('示教连接中断，未记录该操作',true);
    }
  },
  menuPressed(rawEdge) {
    const result=this.owns?this.menu:rawEdge;this.menu=false;return result;
  },
  draw() {
    const camera=document.querySelector('a-scene')?.camera?.el;
    if(!camera)return;
    if(!this.hud) {
      this.canvas=document.createElement('canvas');this.canvas.width=1024;this.canvas.height=170;
      this.texture=new THREE.CanvasTexture(this.canvas);
      this.hud=document.createElement('a-entity');this.hud.setAttribute('position','0 0.66 -1.7');
      const material=new THREE.MeshBasicMaterial({map:this.texture,transparent:true,depthTest:false,depthWrite:false});
      const mesh=new THREE.Mesh(new THREE.PlaneGeometry(1.28,.2125),material);mesh.renderOrder=100;
      this.hud.setObject3D('mesh',mesh);camera.appendChild(this.hud);
    }
    const d=this.data;this.hud.object3D.visible=Boolean(d.enabled||d.phase==='processing');
    if(!this.hud.object3D.visible)return;
    const c=this.canvas.getContext('2d');c.clearRect(0,0,1024,170);
    c.fillStyle='rgba(8,25,36,.92)';c.fillRect(0,0,1024,170);
    c.fillStyle=d.phase==='recording'?'#ff9a88':'#81e9c4';c.font='bold 34px sans-serif';
    const status=d.phase==='recording'?`● 录制中 ${Number(d.elapsed||0).toFixed(1)}秒${d.resetting?' · 正在接入复位':''}`
      :d.phase==='countdown'?`准备录制 ${Number(d.countdown||0).toFixed(1)}秒`
      :d.phase==='processing'?'正在整理与保存…':d.phase==='saved'?'✓ 轨迹已保存，可继续新建动作':d.mode==='trajectory'?'连续轨迹示教':`逐点示教 · ${d.count||0}个姿态`;
    c.fillText(status,24,46,976);
    c.font='24px sans-serif';c.fillStyle='#e6f1f6';
    c.fillText(d.name||'新动作',24,83,976);
    c.fillStyle=this.noticeError&&performance.now()<this.noticeUntil?'#ffac9d':'#b5cbd7';c.font='22px sans-serif';
    const hint=!d.available?'连接中断，录制已暂停'
      :performance.now()<this.noticeUntil?this.notice
      :d.mode==='trajectory'?'长按右摇杆1秒：开始/结束 · 录制中双击：标记'
      :'双击右摇杆：记录姿态 · 单击右摇杆：打开示教菜单';
    c.fillText(hint,24,120,976);c.font='19px sans-serif';c.fillStyle='#91acba';
    c.fillText('底盘接管、摇杆移动及转向保持原功能；底盘位移不录入本动作',24,151,976);
    this.texture.needsUpdate=true;
  },
  items() {
    const d=this.data;return [
      {kind:'teaching-new',label:'＋ 新建下一条动作'},
      {kind:'teaching-mode',label:d.mode==='poses'?'切换为连续轨迹录制':'切换为逐点记录'},
      {kind:'teaching-capture',label:d.mode==='poses'?`记录当前姿态（已有${d.count||0}个）`:'录制中添加关键标记'},
      {kind:'teaching-toggle',label:['recording','countdown'].includes(d.phase)?'■ 结束并保存 / 取消倒计时':'● 开始连续录制（2秒倒计时）'},
      {kind:'teaching-finish',label:'完成示教，回桌面编辑与预演'},
      {kind:'teaching-close',label:'返回原菜单'}];
  }
};
setTimeout(()=>autolifeTeaching.poll(),1200);
