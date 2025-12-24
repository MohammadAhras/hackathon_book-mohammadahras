#!/usr/bin/env node

/**
 * Development Environment Validation Script for Docusaurus Routing Fix
 * Checks that the Docusaurus project is properly set up for routing configuration
 */

const fs = require('fs');
const path = require('path');

console.log('🔍 Validating Docusaurus routing setup...\n');

// Check for required files
const requiredFiles = [
  'package.json',
  'docusaurus.config.js',
  'sidebars.js',
  'docs/intro.md'
];

let allGood = true;

console.log('📋 Checking required files...\n');

for (const file of requiredFiles) {
  const filePath = path.join(__dirname, file);
  if (fs.existsSync(filePath)) {
    console.log(`✅ ${file} - Found`);
  } else {
    console.log(`❌ ${file} - Missing`);
    allGood = false;
  }
}

console.log('\n📋 Checking directory structure...\n');

// Check for required directories
const requiredDirs = [
  'docs',
  'src',
  'static'
];

for (const dir of requiredDirs) {
  const dirPath = path.join(__dirname, dir);
  if (fs.existsSync(dirPath) && fs.statSync(dirPath).isDirectory()) {
    console.log(`✅ ${dir}/ - Found`);
  } else {
    console.log(`❌ ${dir}/ - Missing`);
    allGood = false;
  }
}

console.log('\n📋 Checking package.json dependencies...\n');

try {
  const packageJson = JSON.parse(fs.readFileSync(path.join(__dirname, 'package.json'), 'utf8'));
  const requiredDeps = ['@docusaurus/core', '@docusaurus/preset-classic'];

  for (const dep of requiredDeps) {
    if (packageJson.dependencies?.[dep] || packageJson.devDependencies?.[dep]) {
      console.log(`✅ ${dep} - Found in dependencies`);
    } else {
      console.log(`⚠️  ${dep} - Not found in dependencies`);
    }
  }
} catch (error) {
  console.log(`❌ Error reading package.json: ${error.message}`);
  allGood = false;
}

console.log('\n📋 Checking Docusaurus configuration...\n');

try {
  const configPath = path.join(__dirname, 'docusaurus.config.js');
  if (fs.existsSync(configPath)) {
    const configContent = fs.readFileSync(configPath, 'utf8');
    if (configContent.includes('classic') && configContent.includes('docs:')) {
      console.log('✅ docusaurus.config.js - Contains expected configuration');
    } else {
      console.log('⚠️  docusaurus.config.js - May need configuration updates');
    }
  } else {
    console.log('❌ docusaurus.config.js - File does not exist');
    allGood = false;
  }
} catch (error) {
  console.log(`❌ Error reading docusaurus.config.js: ${error.message}`);
  allGood = false;
}

console.log('\n📋 Checking sidebar configuration...\n');

try {
  const sidebarPath = path.join(__dirname, 'sidebars.js');
  if (fs.existsSync(sidebarPath)) {
    const sidebarContent = fs.readFileSync(sidebarPath, 'utf8');
    if (sidebarContent.includes('tutorialSidebar') && sidebarContent.includes('type: \'doc\'')) {
      console.log('✅ sidebars.js - Contains expected sidebar structure');
    } else {
      console.log('⚠️  sidebars.js - May need configuration updates');
    }
  } else {
    console.log('❌ sidebars.js - File does not exist');
    allGood = false;
  }
} catch (error) {
  console.log(`❌ Error reading sidebars.js: ${error.message}`);
  allGood = false;
}

console.log('\n🎯 Validation complete!\n');

if (allGood) {
  console.log('✅ Environment is properly set up for Docusaurus routing fix!');
  console.log('You can proceed with the routing configuration tasks.');
  process.exit(0);
} else {
  console.log('❌ Some issues were found. Please resolve them before proceeding.');
  process.exit(1);
}