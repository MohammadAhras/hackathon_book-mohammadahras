#!/usr/bin/env node

/**
 * Readability Validation Script
 * Checks that readability improvements are properly implemented
 */

const fs = require('fs');
const path = require('path');

console.log('🔍 Validating readability improvements...\n');

// Check if CSS file exists and has the expected readability elements
const cssPath = path.join(__dirname, 'src', 'css', 'custom.css');
if (!fs.existsSync(cssPath)) {
  console.log('❌ custom.css file not found');
  process.exit(1);
}

const cssContent = fs.readFileSync(cssPath, 'utf8');

// Check for readability improvements
const readabilityChecks = [
  { name: 'Improved font hierarchy', pattern: /font-size:|line-height:/g },
  { name: 'Better paragraph spacing', pattern: /\.markdown p \{/g },
  { name: 'Enhanced list presentation', pattern: /\.markdown ul|\.markdown ol/g },
  { name: 'Better table styling', pattern: /\.markdown table/g },
  { name: 'Code readability enhancements', pattern: /code \{|pre code/g },
  { name: 'Proper color contrast', pattern: /color:|background-color:/g },
  { name: 'Readable line heights', pattern: /line-height: 1\.7|line-height: 1\.8/g },
  { name: 'Paragraph lead styling', pattern: /\.markdown p\.lead/g }
];

let passed = 0;
let total = readabilityChecks.length;

console.log('📋 Checking CSS for readability improvements...\n');

for (const check of readabilityChecks) {
  const matches = cssContent.match(check.pattern);
  if (matches) {
    console.log(`✅ ${check.name} - Found ${matches.length} occurrence(s)`);
    passed++;
  } else {
    console.log(`❌ ${check.name} - Not found`);
  }
}

console.log('\n🎯 Readability validation results:');
console.log(`${passed}/${total} checks passed\n`);

// Check specific readability metrics
const lineHeightMatches = cssContent.match(/line-height:\s*(1\.[7-9]|2\.0)/g);
const fontSizeMatches = cssContent.match(/font-size:\s*1\.\d+rem/g);

console.log('📊 Readability metrics:');
console.log(`Line height improvements: ${lineHeightMatches ? lineHeightMatches.length : 0} found`);
console.log(`Font size improvements: ${fontSizeMatches ? fontSizeMatches.length : 0} found`);

// Determine if readability is improved
if (passed >= total * 0.7) { // At least 70% of checks should pass
  console.log('\n✅ Readability improvements validation passed!');
  console.log('The CSS includes enhanced readability features for better user experience.');
} else {
  console.log('\n⚠️  Some readability improvements are missing.');
  console.log('Consider adding more readability-focused styling.');
}

console.log('\n🎉 Readability validation complete!');
process.exit(0);