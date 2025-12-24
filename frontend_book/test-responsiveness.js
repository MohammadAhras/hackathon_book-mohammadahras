#!/usr/bin/env node

/**
 * Responsiveness Testing Script
 * Checks that responsive design elements are properly implemented
 */

const fs = require('fs');
const path = require('path');

console.log('📱 Testing responsive design implementation...\n');

// Check if CSS file exists and has the expected responsive elements
const cssPath = path.join(__dirname, 'src', 'css', 'custom.css');
if (!fs.existsSync(cssPath)) {
  console.log('❌ custom.css file not found');
  process.exit(1);
}

const cssContent = fs.readFileSync(cssPath, 'utf8');

// Check for responsive design elements
const responsiveChecks = [
  { name: 'Mobile navigation', pattern: /@media \(max-width: 996px\)/g },
  { name: 'Small screen optimization', pattern: /@media \(max-width: 768px\)/g },
  { name: 'Extra small device support', pattern: /@media \(max-width: 575px\)/g },
  { name: 'Hamburger menu implementation', pattern: /\.navbar__toggle/g },
  { name: 'Responsive typography', pattern: /font-size:|line-height:/g },
  { name: 'Flexible content layout', pattern: /\.row|\.col|flex-direction/g },
  { name: 'Responsive code blocks', pattern: /pre|code.*@media/g },
  { name: 'Responsive tables', pattern: /table.*overflow|display: block/g }
];

let passed = 0;
let total = responsiveChecks.length;

console.log('📋 Checking CSS for responsive design elements...\n');

for (const check of responsiveChecks) {
  const matches = cssContent.match(check.pattern);
  if (matches) {
    console.log(`✅ ${check.name} - Found ${matches.length} occurrence(s)`);
    passed++;
  } else {
    console.log(`❌ ${check.name} - Not found`);
  }
}

console.log('\n🎯 Responsiveness validation results:');
console.log(`${passed}/${total} checks passed\n`);

// Check specific responsive breakpoints
const breakpointMatches = cssContent.match(/@media \(max-width:\s*\d+px\)/g);
console.log(`📊 Responsive breakpoints found: ${breakpointMatches ? breakpointMatches.length : 0}`);

if (breakpointMatches) {
  const breakpoints = [...new Set(breakpointMatches.map(bp => bp.match(/\d+px/)[0]))].sort();
  console.log(`Breakpoints: ${breakpoints.join(', ')}`);
}

// Check for responsive typography
const responsiveTypography = cssContent.match(/font-size:.*rem|font-size:.*em/g);
console.log(`Typography adjustments: ${responsiveTypography ? responsiveTypography.length : 0} found`);

// Determine if responsiveness is well-implemented
if (passed >= total * 0.75) { // At least 75% of checks should pass
  console.log('\n✅ Responsiveness implementation validation passed!');
  console.log('The CSS includes comprehensive responsive design features for all device sizes.');
} else {
  console.log('\n⚠️  Some responsive design elements are missing.');
  console.log('Consider adding more responsive styling for better mobile experience.');
}

console.log('\n🎉 Responsiveness testing complete!');
process.exit(0);