const fs=require('fs'),assert=require('assert/strict');
const text=fs.readFileSync(process.argv[2],'utf8');
const start=text.indexOf('function handleDeliveryArrivalInput(');
const end=text.indexOf('\nfunction ',start+10);
assert(start>0&&end>start);
const ui={sessionActive:true,deliveryCardActive:true,baseControlLatched:false,
  deliveryArrivalInputKey:'',deliveryArrivalReady:false,deliveryArrivalDownAt:null,
  deliveryArrivalConsume:false,previous:{a:false,x:false,y:false,b:false},actionPending:false};
const handoff={close_required:true,state:'opened',task_id:'return-task',door_id:'2',
  order_id:'order-1'};
const calls=[];
const make=new Function('navUi','deliveryHandoff','deliveryOfferPending','deliveryDoorPayload',
  'navShowNotice','navigationAction','drawNavigationPanel',
  text.slice(start,end)+'\nreturn handleDeliveryArrivalInput;');
const handle=make(ui,()=>handoff,()=>false,
  ()=>({order_id:'order-1',task_id:'return-task',door_id:'2',client_id:'headset'}),
  ()=>{},(...args)=>{calls.push(args);return Promise.resolve(true);},()=>{});
function tick(ms,a=false,x=false,y=false,b=false){
  const used=handle(a,x,y,b,ms);ui.previous={a,x,y,b};return used;
}
tick(0);tick(1,true);tick(1002,false);
assert.equal(calls.length,1);
assert(calls[0][0].endsWith('/close'));
assert.equal(calls[0][1].confirmed,true);
assert(text.includes("delivery?.task_type === 'RETURN_TO_ROOM'"));
assert(text.includes('Boolean(delivery?.return_load_ready)'));
assert(text.includes("closeRequired ? 'close' : 'open'"));
console.log('PASS: return arrival waits for load readiness; long A closes and requests return to charger');
