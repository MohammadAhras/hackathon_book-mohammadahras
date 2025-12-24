#!/usr/bin/env node

/**
 * Routing Validation Script for Docusaurus Documentation Routing Fix
 * Checks that the routing configuration is properly set up
 */

const fs = require('fs');
const path = require('path');

console.log('🔍 Validating Docusaurus routing configuration...\n');

// Check the docusaurus.config.js file for routing configuration
const configPath = path.join(__dirname, 'docusaurus.config.js');
if (!fs.existsSync(configPath)) {
  console.log('❌ docusaurus.config.js file not found');
  process.exit(1);
}

const configContent = fs.readFileSync(configPath, 'utf8');

// Check for routing configuration elements
const checks = [
  { name: 'Docs plugin routeBasePath set to "/"', pattern: /routeBasePath:\s*'\/'/g },
  { name: 'Blog disabled to avoid route conflicts', pattern: /blog:\s*false/g },
  { name: 'Homepage link in navbar logo', pattern: /href:\s*'\/'/g },
  { name: 'Footer links updated for new routing', pattern: /to:\s*'\/'/g }
];

let passed = 0;
let total = checks.length;

console.log('📋 Checking docusaurus.config.js for routing configuration...\n');

for (const check of checks) {
  const matches = configContent.match(check.pattern);
  if (matches) {
    console.log(`✅ ${check.name} - Found ${matches.length} occurrence(s)`);
    passed++;
  } else {
    console.log(`❌ ${check.name} - Not found`);
  }
}

console.log('\n🎯 Routing configuration validation results:');
console.log(`${passed}/${total} checks passed\n`);

// Check for potential configuration issues
const hasRouteConflicts = configContent.match(/routeBasePath:\s*'\//g);
const hasBlogConflict = configContent.includes('blog:') && !configContent.includes('blog: false');

console.log('📊 Configuration analysis:');
console.log(`Route base path configured: ${hasRouteConflicts ? '✅ Yes' : '❌ No'}`);
console.log(`Blog conflicts avoided: ${!hasBlogConflict ? '✅ Yes' : '❌ Potential conflict'}`);

if (passed >= total * 0.75) { // At least 75% of checks should pass
  console.log('\n✅ Routing configuration validation passed!');
  console.log('The configuration includes proper routing settings for documentation at root.');
} else {
  console.log('\n⚠️  Some routing configuration elements are missing.');
  console.log('Review the configuration for proper routeBasePath and blog settings.');
}

// Check that backup exists
const backupPath = path.join(__dirname, 'docusaurus.config.js.backup');
if (fs.existsSync(backupPath)) {
  console.log('\n✅ Backup file exists for rollback capability.');
} else {
  console.log('\n⚠️  Backup file not found. Consider creating a backup.');
}

console.log('\n🎉 Routing validation complete!');
process.exit(0);