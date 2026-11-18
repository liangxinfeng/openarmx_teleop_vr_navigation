// Run actual hotel controls and A gesture code without a browser or a robot.
const fs = require('fs'), vm = require('vm'), assert = require('assert/strict');
const web = process.argv[2] || 'web';
const ui = {state:{hotel_mission:{active:true,state:'WAITING_FOR_CLEANING',token:'mission:1',
  basket_state:'empty',current_area_id:'round_table_1'},task_dispatch:{state:'awaiting_action'}},
  previous:{a:false},menuVisible:false,selected:0};
const calls = [];
const sandbox = {navUi:ui, console, performance, Date, Math,
  globalThis:{autolifeVrClientId:()=> 'headset',crypto:{randomUUID:()=> String(calls.length)+'-test'}},
  fetch:async()=>({ok:true,json:async()=>({success:true,missions:[{id:'patrol',name:'巡查'}],token:'catalog'})}),
  navigationAction:async(...args)=>{calls.push(args);return true;},
  navShowNotice:()=>{},drawNavigationPanel:()=>{},roundRect:()=>{}};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(`${web}/hotel_controls.js`,'utf8')+';this.controls=hotelControls;',sandbox);
const controls=sandbox.controls;
const text=fs.readFileSync(`${web}/nav_app.js`,'utf8');
const begin=text.indexOf('    const hotelState = navUi.state?.hotel_mission;');
const end=text.indexOf("    if (['navigating', 'navigation_failed'].includes(taskState))",begin);
assert(begin>0&&end>begin);
const gesture=new Function('navUi','hotelControls','taskState','menuOwnsA','aPressed','xPressed',
  'performance','navShowNotice','navigationAction',`let taskAConsumed=false;${text.slice(begin,end)};return taskAConsumed;`);
function tick(ms,a,x=false){gesture(ui,controls,ui.state.task_dispatch.state,ui.menuVisible,a,x,
  {now:()=>ms},()=>{},(...p)=>calls.push(p));ui.previous.a=a;}
tick(0,true);tick(1000,true);tick(1001,false);
assert.equal(calls.length,1);assert.equal(calls[0][0],'/api/tasks/complete');
assert.equal(calls[0][1].token,'mission:1');assert.equal(calls[0][1].client_id,'headset');
tick(1100,false);assert.equal(calls.length,1);
tick(2000,true);tick(3100,true);ui.state.hotel_mission.token='mission:2';tick(3101,false);
assert.equal(calls.length,1,'task change on release must cancel the gesture');
ui.state.hotel_mission.state='PAUSED_FOR_DISPOSAL';
tick(4000,true);tick(5100,true);tick(5200,false);assert.equal(calls.length,1);
ui.menuVisible=true;ui.state.hotel_mission.state='WAITING_FOR_CLEANING';
tick(6000,true);tick(7100,true);tick(7200,false);assert.equal(calls.length,1);
ui.menuVisible=false;ui.state.hotel_mission.state='PICKING_BASKET';
tick(8000,true);tick(9100,true);tick(9200,false);
assert.equal(calls.length,2,'pickup confirmation must use the normal long A gesture');
assert.equal(calls.at(-1)[0],'/api/tasks/complete');
ui.state.hotel_mission.state='PICKUP_FAILED';
tick(10000,true);tick(11100,true);tick(11200,false);
assert.equal(calls.length,2,'failed pickup must require explicit retry');
ui.state.hotel_mission.state='WAITING_FOR_CLEANING';

(async()=>{
  controls.open=true;
  assert(controls.items().some(x=>x.action==='photo'));
  const mark = controls.items().find(x=>x.action==='mark_item' && x.item_label==='water_bottle');
  assert.equal(mark.label,'记一件清理物：水瓶');
  await controls.activate(mark);
  assert.equal(calls.at(-1)[0],'/api/hotel/action');
  assert.equal(calls.at(-1)[1].action,'mark_item');
  assert.equal(calls.at(-1)[1].label,'water_bottle');
  assert(!('item_label' in calls.at(-1)[1]));
  assert(!controls.items().some(x=>x.action==='dispose'));
  ui.state.hotel_mission.basket_state='full';
  const dispose=controls.items().find(x=>x.action==='dispose');
  const before=calls.length;await controls.activate(dispose);
  assert.equal(calls.length,before,'disposal requires explicit menu confirmation');
  assert.equal(controls.pending.payload.arms_secured,true);
  await controls.activate({kind:'hotel-confirm'});
  assert.equal(calls.at(-1)[0],'/api/hotel/action');assert.equal(calls.at(-1)[1].action,'dispose');
  assert(!ui.menuVisible);
  ui.state.hotel_mission.state='PAUSED_FOR_DISPOSAL';
  ui.state.hotel_mission.last_failure={code:'DUMP_FAILED'};controls.open=true;
  assert(controls.items().some(x=>x.action==='retry_dump'));
  ui.state.hotel_mission.active=false;
  await controls.activate({kind:'hotel-open'});
  await controls.activate({kind:'hotel-start',id:'patrol'});
  assert.equal(controls.pending.payload.catalog_token,'catalog');
  assert.equal(controls.pending.payload.start_at_dock_confirmed,true);
  assert(controls.pending.label.includes('前台取篮'));
  console.log('PASS: hotel A completion, stale release, menu ownership, disposal confirmation, retry and start');
})().catch(error=>{console.error(error);process.exitCode=1;});
