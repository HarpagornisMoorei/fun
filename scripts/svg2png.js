// Rasterize SVG files with Chromium (Playwright), for looking at what `mech render` drew.
//
//   node scripts/svg2png.js out.png in.svg [delay-ms]
//
// The optional delay waits that long before the screenshot, which is how a frame of an
// animated SVG is picked (SMIL animations run in real time).
const { chromium } = require('playwright');
const path = require('path');

(async () => {
  const [out, input, delay] = process.argv.slice(2);
  if (!out || !input) { console.error('usage: node scripts/svg2png.js out.png in.svg [delay-ms]'); process.exit(1); }
  const browser = await chromium.launch();
  const page = await browser.newPage({ deviceScaleFactor: 1 });
  await page.goto('file://' + path.resolve(input));
  const size = await page.evaluate(() => {
    const s = document.documentElement;
    return { w: Math.ceil(parseFloat(s.getAttribute('width'))), h: Math.ceil(parseFloat(s.getAttribute('height'))) };
  });
  await page.setViewportSize({ width: size.w, height: size.h });
  if (delay) await page.waitForTimeout(parseInt(delay, 10));
  await page.screenshot({ path: out });
  await browser.close();
})();
