"use strict";
const $ = id => document.getElementById(id);
const newId = () => crypto.randomUUID().replaceAll("-", "");
let client = sessionStorage.getItem("easyfly-client") || newId();
sessionStorage.setItem("easyfly-client", client);
let intent = sessionStorage.getItem("easyfly-intent") || "";
let token = "", state = null, review = null, operation = null, busy = false, lastSeen = 0, generation = 0, operationSeen = 0;
const text = (id, value) => { $(id).textContent = value; };
function notice(message, error=false) { text("notice", message); $("notice").classList.toggle("error", error); }
function clearTelemetry() {
  ["fcu", "armed", "battery", "altitude"].forEach(id => text(id, "—"));
  text("mode", "模式未知"); text("frame", "坐标系未知 / m"); text("position", "X —　Y —　Z —");
  text("observation", "观测未知或已过期"); text("service", "服务未知"); $("service").classList.remove("online");
}
function fresh() { return Boolean(token && state && performance.now() - lastSeen < 2500); }
function controls() {
  const live = fresh(), ready = live && state.vehicle.mission_service_ready;
  const allowed = live && state.role === "operator" && state.execution_enabled;
  $("connect").disabled = busy || Boolean(token); $("disconnect").disabled = !token;
  $("mission").disabled = !ready || busy; $("review").disabled = !ready || busy || !$("mission").value;
  $("confirm").disabled = !allowed || !review || busy;
  $("start").disabled = !ready || !allowed || !review || !$("confirm").checked || busy;
  $("lookup").disabled = !ready || !intent || busy;
  const reply = operation && operation.reply;
  $("cancel").disabled = !ready || !allowed || busy || !reply || !reply.admission_success ||
    !reply.execution_id || !reply.execution_pending || operation.cancel_requested || performance.now()-operationSeen>2500;
}
async function api(path, body) {
  const response = await fetch(path, { method: body ? "POST" : "GET", cache:"no-store",
    headers: {Authorization:"Bearer " + token, ...(body ? {"Content-Type":"application/json"} : {})},
    ...(body ? {body:JSON.stringify(body)} : {}), signal:AbortSignal.timeout(6000) });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || "请求失败：" + response.status);
  return result;
}
function renderStatus(value) {
  if (state && state.boot_id !== value.boot_id) {
    review = null; $("confirm").checked = false;
    notice("服务已重启。请查询原请求确认既有任务状态；新任务需要重新审阅。", true);
  }
  state = value; lastSeen = performance.now();
  text("connection", "已连接"); $("connection").classList.add("online");
  text("platform", value.platform_id); text("account", value.account + " · " + (value.role === "operator" ? "操作员" : "只读"));
  text("clock", value.simulation_time ? "仿真时钟" : "系统时钟");
  text("permission", value.role === "operator" && value.execution_enabled ? "可执行受管任务" : "当前为只读访问");
  const old = $("mission").value;
  $("mission").replaceChildren(...value.missions.map(name => {const option=document.createElement("option");option.value=name;option.textContent=name;return option;}));
  if (value.missions.includes(old)) $("mission").value = old;
  clearTelemetry(); const v=value.vehicle;
  text("observation", v.state_valid && v.flight_valid ? "新鲜观测" : "部分观测未知或已过期");
  text("fcu", v.connected === null ? "—" : v.connected ? "已连接" : "已断开");
  text("armed", v.armed === null ? "—" : v.armed ? "已解锁" : "未解锁");
  text("mode", v.mode || "模式未知"); text("battery", v.battery === null ? "—" : (v.battery*100).toFixed(0) + "%");
  if (v.position) {text("altitude", v.position[2].toFixed(2));text("position", ["X", "Y", "Z"].map((n,i)=>n+" "+v.position[i].toFixed(2)).join("　"));text("frame", (v.frame || "未标注") + " / m");}
  text("service", v.mission_service_ready ? "任务服务就绪" : "任务服务不可用"); $("service").classList.toggle("online", v.mission_service_ready);
}
function renderOperation(value, historical=false) {
  if (!historical) operationSeen = performance.now();
  const prefix = historical ? "上次查询：" : "";
  operation = value; text("execution-name", value.mission_name);
  const r=value.reply; text("execution-id", r && r.execution_id || "待确认");
  if (!r) {text("execution-state", "结果未知：" + (value.error || "请查询原请求"));text("ownership", "未知，不能判定已停止");$("progress").value=0;return;}
  let description = r.status;
  if (r.admission_resolved && !r.admission_success) description = "任务未获准执行：" + r.admission_status;
  else if (!r.admission_resolved) description = "等待任务接纳结果";
  else if (r.execution_pending) description = "任务仍在执行或释放控制权：" + r.status;
  else if (r.execution_id) description = "任务已结束：" + r.status;
  text("execution-state", prefix + description + (r.message ? " · " + r.message : ""));
  text("ownership", r.execution_id ? (r.execution_pending ? prefix + "尚未释放" : prefix + "服务确认已释放") : "尚无已接纳执行");
  $("progress").value = r.feedback_valid ? Math.max(0, Math.min(1, r.current_status.progress)) : 0;
}
async function task(callback) {
  if (busy) return; busy=true; controls(); const epoch=generation;
  try {await callback(epoch);} catch(error) {if(epoch===generation) notice(error.message, true);}
  finally {busy=false;controls();}
}
async function poll() {
  if (!token || busy) return;
  await task(async epoch => {try {const value=await api("/api/status");if(epoch===generation)renderStatus(value);}
    catch(error){if(epoch===generation){lastSeen=0;clearTelemetry();text("connection","连接中断");$("connection").classList.remove("online");notice("连接中断，当前读数已清除。任务不会自动重发或取消。",true);}}});
}
$("login").addEventListener("submit", event => {event.preventDefault();token=$("token").value.trim();$("token").value="";generation++;
  task(async epoch => {try {const value=await api("/api/status");if(epoch!==generation)return;renderStatus(value);notice("已连接。请选择任务并读取原文。");}
    catch(error){if(epoch===generation){token="";state=null;clearTelemetry();}throw error;}});});
$("disconnect").addEventListener("click",()=>{generation++;token="";state=null;review=null;operation=null;lastSeen=0;$("confirm").checked=false;clearTelemetry();text("connection","未连接");$("connection").classList.remove("online");text("account","—");notice("已断开页面连接。已有任务状态保留在服务端，重新连接后可查询原请求。");controls();});
$("mission").addEventListener("change",()=>{review=null;$("confirm").checked=false;text("content","任务已变更，请重新读取。");$("fingerprint").hidden=true;controls();});
$("confirm").addEventListener("change",controls);
$("review").addEventListener("click",()=>task(async epoch=>{const value=await api("/api/review",{mission_name:$("mission").value,client_id:client});if(epoch!==generation)return;review=value;$("confirm").checked=false;text("content",value.content);text("hash",value.content_sha256);text("mission-server",value.server_instance_id);$("fingerprint").hidden=false;notice("已读取任务原文。请核对当前平台与内容，审阅有效期为 2 分钟。");}));
$("start").addEventListener("click",()=>task(async epoch=>{const id=newId();intent=id;sessionStorage.setItem("easyfly-intent",id);const body={boot_id:state.boot_id,client_id:client,request_id:id,review_id:review.review_id};review=null;$("confirm").checked=false;text("execution-state","正在提交；如连接中断，请查询原请求。");const value=await api("/api/start",body);if(epoch===generation){renderOperation(value);notice("启动请求已有记录。请查询执行状态，接纳成功不代表任务已完成。");}}));
$("lookup").addEventListener("click",()=>task(async epoch=>{const value=await api("/api/lookup",{client_id:client,request_id:intent});if(epoch===generation){renderOperation(value);notice("已查询原请求，未重新启动任务。");}}));
$("cancel").addEventListener("click",()=>{if(!window.confirm("取消本页面拥有的这个任务？取消后仍需查询并等待控制权释放。"))return;
  task(async epoch=>{const value=await api("/api/cancel",{boot_id:state.boot_id,client_id:client,request_id:intent,execution_id:operation.reply.execution_id});if(epoch===generation){renderOperation(value);notice("已提交取消，请查询并等待服务确认任务结束。");}});});
setInterval(poll,1000);
setInterval(()=>{if(token && !fresh()){clearTelemetry();text("connection","等待连接恢复");$("connection").classList.remove("online");}if(operation && performance.now()-operationSeen>2500)renderOperation(operation,true);controls();},200);
controls();
