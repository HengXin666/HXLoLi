// Teaching reproduction of OMP's shared/exclusive scheduling rule.
import assert from 'node:assert/strict';
const trace = [];
let barrier = Promise.resolve();
let shared = [];
const tasks = [];
for (const [name, mode, delay] of [['read-A','shared',40],['read-B','shared',5],['debug-step','exclusive',2],['read-C','shared',1]]) {
  const ready = mode === 'exclusive' ? Promise.all([barrier, ...shared]) : barrier;
  const task = ready.then(async () => {
    trace.push(`start:${name}`);
    await new Promise(resolve => setTimeout(resolve, delay));
    trace.push(`end:${name}`);
  });
  tasks.push(task);
  if (mode === 'exclusive') { barrier = task; shared = []; }
  else shared.push(task);
}
await Promise.allSettled(tasks);
assert.deepEqual(trace,['start:read-A','start:read-B','end:read-B','end:read-A','start:debug-step','end:debug-step','start:read-C','end:read-C']);
console.log('SHARED_EXCLUSIVE', JSON.stringify(trace));
