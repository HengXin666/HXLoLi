import assert from 'node:assert/strict';
import { mkdtemp, writeFile, chmod, mkdir, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { Agent } from '@earendil-works/pi-agent-core';
import { Type, type JsonObject, createModels, fauxProvider, fauxAssistantMessage, fauxToolCall } from '@earendil-works/pi-ai';
import { git, review, acquire } from './review.ts';
const models = createModels(); const faux = fauxProvider(); models.setProvider(faux.provider);
const streamFn = models.streamSimple.bind(models);
const pause = (ms: number) => new Promise(r => setTimeout(r, ms));
for (const sequential of [false, true]) {
  const trace: string[] = []; const resultIds: string[] = []; let running = 0; let peak = 0;
  const agent = new Agent({ streamFn, initialState: { model: faux.getModel(), tools: [
    { name: 'read', label: 'Read', description: 'Fixture read', parameters: Type.Object({ delay: Type.Number() }),
      executionMode: sequential ? 'sequential' : 'parallel',
      async execute(id, args) {
        trace.push(`start:${id}`); peak = Math.max(peak, ++running);
        await pause((args as {delay:number}).delay); running--; trace.push(`end:${id}`);
        return { content: [{type:'text',text:id}],details:{} };
      } },
  ] } });
  agent.subscribe(e => { if(e.type === 'message_end' && e.message.role === 'toolResult') resultIds.push(e.message.toolCallId); });
  faux.setResponses([
    fauxAssistantMessage([fauxToolCall('read',{delay:80},{id:'slow'}),fauxToolCall('read',{delay:5},{id:'fast'})], {stopReason:'toolUse'}),
    fauxAssistantMessage('Done'),
  ]);
  await agent.prompt('Read both');
  assert.equal(peak, sequential ? 1 : 2); assert.deepEqual(resultIds,['slow','fast']);
  console.log(sequential ? 'SEQUENTIAL' : 'PARALLEL', JSON.stringify({trace,resultIds,peak}));
}
const cwd = await mkdtemp(join(tmpdir(),'pi-fixture-'));
try {
  await git(cwd,['init']);
  await writeFile(join(cwd,'sum.ts'),'export const sum = (a: number, b: number) => a + b;\n');
  await git(cwd,['add','.']);
  await git(cwd,['-c','user.name=Fixture','-c','user.email=fixture@example.invalid','commit','-m','base']);
  const base=(await git(cwd,['rev-parse','HEAD'])).trim();
  await writeFile(join(cwd,'sum.ts'),'export const sum = (a: number, b: number) => a - b;\n');
  await git(cwd,['add','.']);
  await git(cwd,['-c','user.name=Fixture','-c','user.email=fixture@example.invalid','commit','-m','regression']);
  const head=(await git(cwd,['rev-parse','HEAD'])).trim();
  await git(cwd,['update-ref','refs/pull/1/head',head]);
  const bin=join(cwd,'mock-bin'); await mkdir(bin);
  await writeFile(join(bin,'gh'), `#!/bin/sh\nprintf '%s\\n' '${JSON.stringify({base:{sha:base},head:{sha:head}})}'\n`);
  await chmod(join(bin,'gh'),0o755);
  const saved={...process.env};
  process.env.PATH=bin+':'+process.env.PATH;
  process.env.GIT_CONFIG_COUNT='1';
  process.env.GIT_CONFIG_KEY_0='url.'+cwd+'.insteadOf';
  process.env.GIT_CONFIG_VALUE_0='https://github.com/fixture/repo.git';
  const acquired=await acquire('fixture/repo#1');
  for(const k of Object.keys(process.env)) if(!(k in saved)) delete process.env[k];
  Object.assign(process.env,saved);
  assert.equal(acquired.head,head); assert.equal(acquired.base,base);
  await rm(acquired.cwd,{recursive:true,force:true});
  console.log('ACQUIRE_FIXTURE: PR -> isolated bare repository -> verified head SHA -> merge-base');
  const patch=await git(cwd,['diff','--binary',base,head]);
  await writeFile(join(cwd,'fix.patch'),patch);
  await writeFile(join(cwd,'sum.ts'),await git(cwd,['show',base+':sum.ts']));
  await git(cwd,['apply','--check','fix.patch']);
  await git(cwd,['apply','fix.patch']);
  assert.equal(await git(cwd,['diff',head,'--','sum.ts']),'');
  console.log('PATCH_ROUNDTRIP: generated diff -> check -> apply -> matches head');
  const tool = (name:string,args:JsonObject) => fauxAssistantMessage(fauxToolCall(name,args),{stopReason:'toolUse'});
  const finding={path:'sum.ts',line:1,priority:1,title:'Restore addition',evidence:'sum(2, 1) now returns 1 instead of 3'};
  faux.setResponses([
    fauxAssistantMessage([fauxToolCall('read_diff',{path:'sum.ts',offset:1,limit:100}),fauxToolCall('read_file',{path:'sum.ts',revision:'head',offset:1,limit:20})],{stopReason:'toolUse'}),
    tool('finish_review',{findings:[{...finding,line:99}]}),
    tool('finish_review',{findings:[finding]}),
  ]);
  const output = await review({cwd,base,head,ref:'fixture/repo#1'},faux.getModel(),streamFn);
  assert.equal((output as any).findings[0].line,1);
  console.log('REVIEW_FIXTURE',JSON.stringify(output));
  console.log('PASS: parallel execution, stable transcript ordering, sequential override, invalid-line retry, validated report');
} finally { await rm(cwd,{recursive:true,force:true}); }
