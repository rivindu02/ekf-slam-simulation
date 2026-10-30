// Exercise all embedded replay frames and controls with a minimal DOM/canvas harness.
// Run from the project root: node scripts/validate_guide_replay.cjs
// This checks replay logic and finite drawing coordinates; it does not test browser layout.
const fs=require('fs'), vm=require('vm'), assert=require('node:assert/strict');
const html=fs.readFileSync('PROJECT_GUIDE.html','utf8');
const data=JSON.parse(html.match(/<script id="projectData" type="application\/json">([\s\S]*?)<\/script>/)[1]);
const script=html.match(/<script>\s*('use strict';[\s\S]*?)<\/script>/)[1];
const elements=new Map();let drawingCalls=0;
const context=new Proxy({}, {get(target,key){if(key in target)return target[key];return (...args)=>{drawingCalls++;assert(args.every(x=>typeof x!=='number'||Number.isFinite(x)),`non-finite canvas argument in ${key}`)}},set(target,key,v){target[key]=v;return true}});
for(const tag of html.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)){
  const [source,id]=tag;
  const element={value:source.match(/\bvalue="([^"]*)"/)?.[1]||'',checked:/\bchecked\b/.test(source),disabled:false,textContent:'',style:{},width:760,height:660,getContext:()=>context,click(){if(!this.disabled)this.onclick?.()}};
  let text='';Object.defineProperty(element,'textContent',{get:()=>text,set:v=>text=String(v)});
  elements.set(id,element);
}
elements.get('scenario').value='baseline';elements.get('speed').value='4';
const sandbox={DATA:data,document:{getElementById:id=>{assert(elements.has(id),`Missing DOM id ${id}`);return elements.get(id)},addEventListener(){}},requestAnimationFrame(){}};
vm.createContext(sandbox);
const prefix=script.slice(script.indexOf('const $='),script.indexOf('const safeStore='));
const replay=script.slice(script.indexOf('// Shared canvas helpers.'),script.indexOf('// Actual correlation matrix'));
vm.runInContext(prefix+'\n'+replay,sandbox);
const result=vm.runInContext(`(()=>{
  const check=(ok,msg)=>{if(!ok)throw Error(msg)};
  check($('showMap').checked,'historical map hidden by default');
  check($('seenValue').textContent==='2 / 10','initial frame exposes future map');
  check($('associationReadout').textContent.includes('New IDs: 0, 1'),'first sightings missing');
  $('jumpRevisit').click();
  check($('timeValue').textContent==='42.1 s','incorrect first closure time');
  check($('associationReadout').textContent.includes('Revisited IDs: 0'),'returning ID missing');
  check($('seenValue').textContent==='10 / 10','map has duplicate entries');
  check($('revisitValue').textContent==='1','closure count incorrect');
  const firstClosure=$('associationReadout').textContent;
  for(const name of Object.keys(DATA.replays)){
    $('scenario').value=name;$('scenario').onchange();
    const r=DATA.replays[name];
    for(let k=0;k<r.time.length;k++){replayIndex=k;drawReplay()}
    for(const id of ['showMap','showRays','showFinal']){
      $(id).checked=!$(id).checked;$(id).onchange();$(id).checked=!$(id).checked;$(id).onchange();
    }
    if(name==='no_landmark_reuse'){
      check($('jumpRevisit').disabled,'no-reuse enabled closure jump');
      check($('updatesValue').textContent==='0','no-reuse has corrections');
      check($('revisitValue').textContent==='0','no-reuse has loop closures');
    }
  }
  $('scenario').value='sensor_dropout';$('scenario').onchange();
  replayIndex=DATA.replays.sensor_dropout.time.findIndex(t=>t>=40);drawReplay();
  check($('sensorStatus').textContent==='Sensor outage','dropout displays active sensor');
  check(DATA.replays.sensor_dropout.measurements[replayIndex].every(z=>z===null),'fabricated dropout measurements');
  $('scenario').value='baseline';$('scenario').onchange();
  replayIndex=DATA.replays.baseline.time.length-1;drawReplay();
  check($('errorValue').textContent==='0.032 m','final robot error changed');
  const final={error:$('errorValue').textContent,updates:$('updatesValue').textContent,revisits:$('revisitValue').textContent};
  replayIndex=DATA.replays.baseline.time.length-2;playing=true;lastFrame=1000;accumulator=0;$('speed').value='1';
  replayTick(1200);check(playing,'final sample reached too early');
  replayTick(1400);check(!playing,'replay failed to stop at actual final timestamp');
  $('resetReplay').click();check($('timeValue').textContent==='0.1 s','restart failed');
  return {firstClosure,final,scenarios:Object.keys(DATA.replays).length,frames:Object.values(DATA.replays).reduce((n,r)=>n+r.time.length,0)};
})()`,sandbox);
console.log(JSON.stringify({...result,drawingCalls},null,2));
