// Takes the viewer screenshots in figures/08_simulator_viewer.png from a running simulator.
//
//   .venv/bin/python -m tests.policy.simulator cross --marker-id 3 --no-browser --port 8000   (feature/car-simulator)
//   THREE_DIR=<unpacked npm three@0.160.0>/package node docs/logbook/long/viewer_screenshots.js OUTDIR
//
// It needs Playwright. THREE_DIR serves three.js locally for a machine that can't reach cdn.jsdelivr.net; leave it
// unset to load it from the CDN as the viewer normally does. The run is paused before each shot, so the times in the
// figure's captions are the simulated times shown in the viewer's top right corner.
const { chromium } = require('playwright');
const fs = require('fs');
const path = require('path');

const PLAN = [
  { control: { op: 'restart' } }, { wait: 3700 }, { control: { op: 'pause' } }, { wait: 400 },
  { key: 'Digit1' }, { wait: 400 }, { shot: 'cross_chase_approach.png' },
  { key: 'Digit2' }, { wait: 500 }, { shot: 'cross_camera_approach.png' },
  { control: { op: 'track', value: 'obstacle_lane' } }, { wait: 600 }, { control: { op: 'restart' } },
  { control: { op: 'resume' } }, { key: 'Digit1' }, { wait: 5200 }, { control: { op: 'pause' } }, { wait: 400 },
  { shot: 'obstacle_chase.png' },
  { control: { op: 'track', value: 'aruco_lane' } }, { wait: 600 }, { control: { op: 'restart' } },
  { control: { op: 'resume' } }, { key: 'Digit1' }, { wait: 6500 }, { control: { op: 'pause' } }, { wait: 400 },
  { shot: 'aruco_chase.png' },
];

(async () => {
  const outDir = process.argv[2] || '.';
  const port = process.env.PORT || 8000;
  const browser = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'] });
  const page = await browser.newPage({ viewport: { width: 1600, height: 900 }, deviceScaleFactor: 2 });
  if (process.env.THREE_DIR) {
    await page.route('https://cdn.jsdelivr.net/npm/three@0.160.0/**', (route) => {
      const file = path.join(process.env.THREE_DIR, route.request().url().split('three@0.160.0/')[1]);
      if (fs.existsSync(file)) route.fulfill({ status: 200, contentType: 'application/javascript', body: fs.readFileSync(file) });
      else route.abort();
    });
  }
  for (let i = 0; i < 60; i++) {
    try { await page.goto(`http://127.0.0.1:${port}/`, { waitUntil: 'load' }); break; } catch (e) { await page.waitForTimeout(250); }
  }
  await page.waitForTimeout(1000);
  for (const step of PLAN) {
    if (step.control) {
      await page.evaluate((body) => fetch('/control', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                                        body: JSON.stringify(body) }), step.control);
    }
    if (step.key) await page.evaluate((code) => window.dispatchEvent(new KeyboardEvent('keydown', { code })), step.key);
    if (step.wait) await page.waitForTimeout(step.wait);
    if (step.shot) { await page.screenshot({ path: path.join(outDir, step.shot) }); console.log('wrote', step.shot); }
  }
  await browser.close();
})();
