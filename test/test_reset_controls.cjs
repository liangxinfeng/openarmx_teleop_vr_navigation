const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('node:assert/strict');

const sourcePath = process.argv[2] || path.join(__dirname, '..', 'web', 'reset_controls.js');
const source = fs.readFileSync(sourcePath, 'utf8');

function createHarness(action) {
  const calls = [];
  const notices = [];
  let uuid = 0;
  const sandbox = {
    Promise,
    Date,
    Math,
    console,
    crypto: { randomUUID: () => 'operation-' + (++uuid) },
    autolifeVrClientId: () => 'test-client',
    navigationAction: (endpoint, payload) => {
      calls.push({ endpoint, payload });
      return action ? action(endpoint, payload, calls.length - 1) : true;
    },
    navShowNotice: (message) => notices.push(message),
    drawNavigationPanel: () => {}
  };
  sandbox.globalThis = sandbox;
  vm.runInNewContext(source, sandbox, { filename: sourcePath });
  return { controls: sandbox.autolifeResetControls, calls, notices };
}

function tick(harness, now, values = {}) {
  return harness.controls.handle({
    y: false,
    b: false,
    x: false,
    a: false,
    axis: { x: 0, y: 0 },
    now,
    ...values
  });
}

function openMenu(harness, start = 0) {
  tick(harness, start, { y: true, b: true });
  tick(harness, start + 999, { y: true, b: true });
  tick(harness, start + 1000, { y: true, b: true });
  assert.equal(harness.controls.ownsInput(), true);
  tick(harness, start + 1010);
}

function enterConfirm(harness, start) {
  tick(harness, start, { a: true });
  tick(harness, start + 10);
}

async function flushPromises() {
  await Promise.resolve();
  await Promise.resolve();
}

async function testFilterFirstYBPacket() {
  const harness = createHarness();
  const head = { position: { x: 1, y: 2, z: 3 } };
  const leftPosition = { x: 4, y: 5, z: 6 };
  const packet = {
    head,
    leftController: {
      position: leftPosition, yButton: 1, xButton: 1, gripActive: false,
      gripValue: 0.1, trigger: 0.8, thumbstick: { x: 0.5, y: 1, pressed: 1 }
    },
    rightController: {
      position: { x: 7, y: 8, z: 9 }, bButton: 1, aButton: 1,
      gripActive: false, gripValue: 0.12, trigger: 1, thumbstick: { x: -1, y: 0.4, pressed: 1 }
    }
  };
  const filtered = harness.controls.filterPacket(packet);
  assert.equal(filtered, packet);
  assert.equal(harness.controls.ownsInput(), true);
  assert.equal(packet.resetMenuActive, true);
  assert.equal(packet.resetMenuGripsReleased, true);
  assert.equal(packet.head, head);
  assert.equal(packet.leftController.position, leftPosition);
  for (const controller of [packet.leftController, packet.rightController]) {
    for (const key of ['xButton', 'yButton', 'aButton', 'bButton']) assert.equal(controller[key], 0);
    assert.equal(controller.gripActive, false);
    assert.equal(controller.grip, false);
    assert.equal(controller.gripValue, 0);
    assert.equal(controller.trigger, 0);
    assert.equal(controller.thumbstick.x, 0);
    assert.equal(controller.thumbstick.y, 0);
    assert.equal(controller.thumbstick.pressed, 0);
  }

  const activeGrip = {
    leftController: { yButton: 1, gripActive: true, gripValue: 0.4 },
    rightController: { bButton: 1, gripActive: false, gripValue: 0 }
  };
  harness.controls.filterPacket(activeGrip);
  assert.equal(activeGrip.resetMenuGripsReleased, false);

  const rootPacket = {
    yButton: 1, bButton: 1, xButton: 1, aButton: 1,
    gripActive: true, grip: true, gripValue: 0.7, trigger: 0.9,
    thumbstick: { x: 1, y: 1, pressed: 1 },
    leftController: { gripActive: false, gripValue: 0 },
    rightController: { gripActive: false, gripValue: 0 }
  };
  harness.controls.filterPacket(rootPacket);
  assert.equal(rootPacket.yButton, 0);
  assert.equal(rootPacket.bButton, 0);
  assert.equal(rootPacket.xButton, 0);
  assert.equal(rootPacket.aButton, 0);
  assert.equal(rootPacket.gripActive, false);
  assert.equal(rootPacket.grip, false);
  assert.equal(rootPacket.gripValue, 0);
  assert.equal(rootPacket.trigger, 0);
  assert.equal(rootPacket.thumbstick.x, 0);
  assert.equal(rootPacket.thumbstick.y, 0);
  assert.equal(rootPacket.resetMenuGripsReleased, true);
}

function testShortChordReleaseOrders() {
  const releaseSequences = [
    [{ y: false, b: true }, { y: false, b: false }],
    [{ y: true, b: false }, { y: false, b: false }]
  ];
  for (const releases of releaseSequences) {
    const harness = createHarness();
    assert.equal(tick(harness, 0, { y: true, b: true }), true);
    assert.equal(tick(harness, 400, { y: true, b: true }), true);
    assert.equal(tick(harness, 450, releases[0]), true);
    assert.equal(tick(harness, 500, releases[1]), true);
    assert.equal(tick(harness, 510), false);
    assert.equal(harness.calls.length, 0);
  }
}

async function testDoubleAPressAndBusyLock() {
  let resolveRequest;
  const harness = createHarness(() => new Promise((resolve) => { resolveRequest = resolve; }));
  openMenu(harness);
  tick(harness, 1020, { a: true });
  assert.equal(harness.calls.length, 0);
  tick(harness, 1030);
  tick(harness, 1040, { a: true });
  assert.equal(harness.calls.length, 1);
  const request = harness.calls[0];
  assert.equal(request.endpoint, '/api/reset/arms');
  assert.equal(request.payload.client_id, 'test-client');
  assert.equal(request.payload.operation_id, 'operation-1');
  assert.equal(request.payload.scope, 'right');
  assert.equal(request.payload.confirmed, true);
  assert.equal(request.payload.preserve_grippers, true);

  tick(harness, 1070);
  tick(harness, 1080, { a: true });
  tick(harness, 1090, { x: true, y: true, b: true });
  assert.equal(harness.calls.length, 1);
  assert.equal(harness.controls.ownsInput(), true);
  resolveRequest(true);
  await flushPromises();
  assert.equal(harness.controls.ownsInput(), true);
  assert.equal(tick(harness, 1100), true);
  assert.equal(harness.controls.ownsInput(), false);
  assert.equal(tick(harness, 1110), false);
}

async function testAllScopesPreserveGrippers() {
  for (const choice of [
    { scope: 'right', moves: 0 },
    { scope: 'left', moves: 1 },
    { scope: 'both', moves: 2 }
  ]) {
    const harness = createHarness(() => true);
    openMenu(harness);
    let now = 1020;
    for (let step = 0; step < choice.moves; step += 1) {
      tick(harness, now++, { axis: { y: 1 } });
      tick(harness, now++);
    }
    enterConfirm(harness, now);
    now += 20;
    tick(harness, now++, { a: true });
    await flushPromises();
    assert.equal(harness.calls.length, 1);
    assert.equal(harness.calls[0].payload.scope, choice.scope);
    assert.equal(harness.calls[0].payload.preserve_grippers, true);
  }
}

function testBackAndLongXClose() {
  const harness = createHarness();
  openMenu(harness);
  enterConfirm(harness, 1020);
  tick(harness, 1040, { b: true });
  tick(harness, 1050);
  tick(harness, 1060, { a: true });
  tick(harness, 1070);
  assert.equal(harness.calls.length, 0);
  tick(harness, 1080, { a: true });
  assert.equal(harness.calls.length, 1);

  const cancelHarness = createHarness();
  openMenu(cancelHarness);
  tick(cancelHarness, 1020, { x: true });
  assert.equal(tick(cancelHarness, 2020, { x: true }), true);
  assert.equal(cancelHarness.controls.ownsInput(), true);
  assert.equal(tick(cancelHarness, 2021), true);
  assert.equal(cancelHarness.controls.ownsInput(), false);
  assert.equal(tick(cancelHarness, 2030), false);
  assert.equal(cancelHarness.calls.length, 0);
}

function testChordedAIsIgnored() {
  const harness = createHarness();
  openMenu(harness);
  tick(harness, 1020, { a: true, b: true });
  tick(harness, 1030);
  tick(harness, 1040, { a: true, y: true });
  tick(harness, 1050);
  tick(harness, 1060, { a: true, b: true });
  assert.equal(harness.calls.length, 0);
  tick(harness, 1070);
  tick(harness, 1080, { a: true });
  tick(harness, 1090);
  tick(harness, 1100, { a: true });
  assert.equal(harness.calls.length, 1);
}

async function testFailureNeedsAllButtonsReleasedAndNewOperationId() {
  const pending = [];
  const harness = createHarness(() => new Promise((resolve, reject) => pending.push({ resolve, reject })));
  openMenu(harness);
  enterConfirm(harness, 1020);
  tick(harness, 1040, { a: true });
  tick(harness, 1050);
  tick(harness, 1060, { a: true });
  assert.equal(harness.calls.length, 1);

  tick(harness, 1070, { b: true });
  pending[0].reject(new Error('offline'));
  await flushPromises();
  assert.equal(harness.notices.length, 1);
  tick(harness, 1080, { a: true });
  assert.equal(harness.calls.length, 1);
  tick(harness, 1090);
  assert.equal(tick(harness, 1100), true);
  tick(harness, 1110, { a: true });
  assert.equal(harness.calls.length, 2);
  assert.equal(harness.calls[1].payload.operation_id, 'operation-2');
  assert.notEqual(harness.calls[0].payload.operation_id, harness.calls[1].payload.operation_id);
  pending[1].resolve(true);
  await flushPromises();
}

async function main() {
  await testFilterFirstYBPacket();
  testShortChordReleaseOrders();
  await testDoubleAPressAndBusyLock();
  await testAllScopesPreserveGrippers();
  testBackAndLongXClose();
  testChordedAIsIgnored();
  await testFailureNeedsAllButtonsReleasedAndNewOperationId();
  console.log('reset controls VM tests passed: chord ownership, confirmation, scopes, gripper preservation, filter, busy/retry, B and X');
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
