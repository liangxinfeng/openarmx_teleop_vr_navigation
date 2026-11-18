const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const root=process.argv[2]||'web';
const source=fs.readFileSync(`${root}/nav_app.js`,'utf8');
const begin=source.indexOf('    const resetWasOwned =');
const end=source.indexOf('    // Chord ownership lasts',begin);
assert(begin>0&&end>begin);
const ui={previous:{},visible:false,baseControlLatched:true};
const calls=[];
let consumed=false,owned=false;
const controls={ownsInput:()=>owned,handle:input=>{calls.push(['handle',input]);return consumed;}};
const globals={autolifeResetControls:controls,autolifeHeadFollowControl:{enabled:()=>true,setEnabled:enabled=>{calls.push(['head',enabled]);}}};
const run=new Function('navUi','globalThis','yPressed','bPressed','xPressed','aPressed','leftButtons','rightButtons','right','performance','setBaseControlLatched','sendBaseDrive','setNavigationView',source.slice(begin,end)+'\nreturn "ordinary";');
function tick(y,b,x=false,a=false){
 return run(ui,globals,y,b,x,a,[{},{},{},{pressed:true}],[{},{},{},{pressed:true}],{axes:[0,0,0,.8]},
   {now:()=>10},enabled=>{ui.baseControlLatched=enabled;calls.push(['base',enabled]);},enabled=>calls.push(['drive',enabled]),enabled=>{ui.visible=enabled;});
}
assert.equal(tick(false,false),'ordinary');
consumed=true;
assert.equal(tick(true,true),undefined);
assert.equal(ui.baseControlLatched,false);
assert(ui.visible);
assert(ui.taskSkipCancelled&&ui.taskHoldCancelled&&ui.taskRejectCancelled);
assert(ui.previous.leftStick&&ui.previous.rightStick,'modal owns thumbstick edges');
assert(calls.some(x=>x[0]==='head'&&x[1]===false));
assert(calls.some(x=>x[0]==='drive'&&x[1]===false));
assert(source.indexOf('const resetWasOwned')<source.indexOf('if (navUi.exitConfirm)',begin));
assert(source.indexOf('const resetWasOwned')<source.indexOf('if (handleDeliveryOfferInput',begin));
const transport=fs.readFileSync(`${root}/vr_app.js`,'utf8');
const sendBegin=transport.indexOf('function sendRealtimePacket(payload)');
const sendEnd=transport.indexOf('\nfunction ',sendBegin+10);
assert(transport.slice(sendBegin,sendEnd).indexOf('filterPacket(payload)')<transport.slice(sendBegin,sendEnd).indexOf('JSON.stringify(payload)'));
assert(transport.includes('if (globalThis.autolifeResetControls?.ownsInput()) return;'));
assert(transport.includes('this.resetDesktopModeChord();'));
const html=fs.readFileSync(`${root}/index.html`,'utf8');
assert(html.indexOf('reset_controls.js')<html.indexOf('nav_app.js'));
console.log('PASS: reset modal owns task/delivery/base/head input; both transports filter before serialization');
