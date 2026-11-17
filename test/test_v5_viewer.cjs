// Exercise the real connection function without opening sockets or a robot.
const fs=require('fs'), assert=require('assert/strict');
const s=fs.readFileSync(process.argv[2],'utf8');
const start=s.indexOf('function connectWebSocket() {');
const end=s.indexOf('\nglobalThis.autolifeVrControlRequested',start);
assert(start>=0&&end>start);
let opened=0;
class FakeSocket { static CONNECTING=0; static OPEN=1; constructor(url){opened++;this.url=url;this.readyState=0;} }
const st={controlRequested:true,observerOnly:false,vrControlCapable:false,xrSession:null,websocket:null,reconnectTimer:null};
const connect=new Function('state','WebSocket','websocketUrl','window','setStatus','refreshHardwareStatus','connectRealtimeChannel','startCameraFeed','cameraId','closeRealtimeChannel','renderHardwareControl',s.slice(start,end)+'\nreturn connectWebSocket;')(
 st,FakeSocket,()=> 'wss://test/ws',{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{});
connect();assert.equal(opened,0,'ordinary desktop must not claim VR slot');
st.observerOnly=true;st.vrControlCapable=true;connect();assert.equal(opened,0,'explicit viewer must not claim VR slot');
st.observerOnly=false;connect();assert.equal(opened,1,'capable headset can claim slot');
connect();assert.equal(opened,1,'existing socket is not duplicated');
st.websocket=null;st.vrControlCapable=false;st.xrSession={};connect();assert.equal(opened,2,'XR session recovers from failed capability probe');
st.websocket=null;st.controlRequested=false;connect();assert.equal(opened,2,'explicit exit prevents reconnect');
assert(s.includes('await detectVrControlCapability()'));
assert(s.includes('void startLatestJpegStream(id)'));
console.log('V5 navigation viewer/control ownership: passed');
