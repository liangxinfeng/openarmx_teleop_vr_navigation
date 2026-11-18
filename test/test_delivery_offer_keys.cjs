const fs=require('fs'),assert=require('assert/strict');
const text=fs.readFileSync(process.argv[2],'utf8');
const start=text.indexOf('function handleDeliveryOfferInput(');
const end=text.indexOf('\nfunction ',start+10);
assert(start>0&&end>start);
const make=new Function('navUi','deliveryOfferPending','deliveryHandoff','navShowNotice','navigationAction',
 text.slice(start,end)+'; return handleDeliveryOfferInput;');
globalThis.autolifeVrClientId=()=> 'headset';
function scenario(kind='pickup'){
 const ui={previous:{},actionPending:false,menuVisible:true},calls=[];
 let pending=true,key='test';
 const handle=make(ui,()=>pending,()=>({offer:{order:'order-1',key,kind}}),()=>{},(...c)=>calls.push(c));
 return {ui,calls,set pending(v){pending=v},set key(v){key=v},
 tick(ms,a=false,x=false,y=false,b=false){const used=handle(a,x,y,b,ms);ui.previous={a,x,y,b};return used;}};
}
let s=scenario();s.tick(0,true);s.tick(2000,true);assert.equal(s.calls.length,0);
s.tick(2100);s.tick(2200,true);s.tick(3199,true);assert.equal(s.calls.length,0);
s.tick(3200,true);assert.equal(s.calls.length,1);assert(s.calls[0][0].endsWith('/accept'));
assert.equal(s.calls[0][1].order,'order-1');assert.equal(s.calls[0][1].offer_key,'test');assert.equal(s.ui.menuVisible,false);
s.pending=false;assert(s.tick(4200,true));assert.equal(s.calls.length,1);
assert(s.tick(4300));assert.equal(s.tick(4400),false);
s=scenario();s.tick(0);s.tick(1,true);s.tick(900);assert.equal(s.calls.length,0);
s=scenario();s.tick(0);s.tick(1,false,true);assert(s.calls[0][0].endsWith('/reject'));
s=scenario();s.tick(0);s.tick(1,true);s.tick(500,true,true);s.tick(1700,true);assert.equal(s.calls.length,0);
s=scenario();s.tick(0);s.tick(1,true);s.key='changed';s.tick(1200,true);assert.equal(s.calls.length,0);
s=scenario();s.tick(0);s.tick(1,true);s.ui.actionPending=true;s.tick(1200,true);assert.equal(s.calls.length,0);
s=scenario('return');s.tick(0);s.tick(1,true);s.tick(1001,true);assert.equal(s.calls.length,1);
assert.equal(s.calls[0][1].offer_key,'test');
const hook=text.indexOf('if (handleDeliveryOfferInput(aPressed');
assert(hook>0 && hook<text.indexOf("const taskState = String(navUi.state?.task_dispatch",hook));
assert(hook<text.indexOf('if (navUi.menuVisible)',hook));
assert(!text.includes('// Observe release first:'));
console.log('PASS: phase-keyed pickup/return offers, long A once, X reject, menu priority and held-key gating');
