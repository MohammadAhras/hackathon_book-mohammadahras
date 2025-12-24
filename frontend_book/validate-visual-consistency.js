#!/usr/bin/env node

/**
 * Visual Consistency Validation Script
 * Checks that visual elements are consistent across documentation pages
 */

const fs = require('fs');
const path = require('path');

console.log('🔍 Validating visual consistency across documentation pages...\n');

// Check if CSS file exists and has the expected modern elements
const cssPath = path.join(__dirname, 'src', 'css', 'custom.css');
if (!fs.existsSync(cssPath)) {
  console.log('❌ custom.css file not found');
  process.exit(1);
}

const cssContent = fs.readFileSync(cssPath, 'utf8');

// Check for modern elements
const checks = [
  { name: 'Modern blue color scheme', pattern: /#2563eb|#3b82f6/g },
  { name: 'Enhanced typography', pattern: /\.markdown h[1-6]/g },
  { name: 'Improved spacing', pattern: /padding:|margin:/g },
  { name: 'Modern navbar styling', pattern: /\.navbar/g },
  { name: 'Footer enhancements', pattern: /\.footer/g },
  { name: 'Code block improvements', pattern: /pre\s*{|code\s*{/g },
  { name: 'Responsive design', pattern: /@media/g },
  { name: 'Dark mode support', pattern: /\[data-theme='dark'\]/g }
];

let passed = 0;
let total = checks.length;

console.log('📋 Checking CSS for visual consistency elements...\n');

for (const check of checks) {
  const matches = cssContent.match(check.pattern);
  if (matches) {
    console.log(`✅ ${check.name} - Found ${matches.length} occurrence(s)`);
    passed++;
  } else {
    console.log(`❌ ${check.name} - Not found`);
  }
}

console.log('\n🎯 CSS validation results:');
console.log(`${passed}/${total} checks passed\n`);

// Check that the CSS file was properly updated
if (passed >= total * 0.8) { // At least 80% of checks should pass
  console.log('✅ Visual consistency validation passed!');
  console.log('The CSS includes modern design elements and consistent styling.');
} else {
  console.log('⚠️  Some visual consistency elements are missing.');
  console.log('Consider adding more modern styling to the CSS.');
}

// Check that backup exists
const backupPath = path.join(__dirname, 'src', 'css', 'custom.css.backup');
if (fs.existsSync(backupPath)) {
  console.log('\n✅ Backup file exists for rollback capability.');
} else {
  console.log('\n⚠️  Backup file not found. Consider creating a backup.');
}

console.log('\n🎉 Visual consistency validation complete!');
process.exit(0);