#!/usr/bin/env node

/**
 * Development Environment Validation Script for Docusaurus UI Fixes
 * Validates that the Docusaurus project is properly set up for UI improvements
 */

const fs = require('fs');
const path = require('path');

console.log('🔍 Validating development environment for Docusaurus UI fixes...\n');

// Check for required files
const requiredFiles = [
  'package.json',
  'docusaurus.config.js',
  'sidebars.js',
  'src/css/custom.css',
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
  'src/css',
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
    const hasDocsPlugin = configContent.includes('docs:');
    const hasClassicPreset = configContent.includes('classic');
    const hasCustomCss = configContent.includes('custom.css');

    if (hasDocsPlugin) console.log('✅ docusaurus.config.js - Contains docs plugin configuration');
    else { console.log('❌ docusaurus.config.js - Missing docs plugin configuration'); allGood = false; }

    if (hasClassicPreset) console.log('✅ docusaurus.config.js - Contains classic preset');
    else { console.log('❌ docusaurus.config.js - Missing classic preset'); allGood = false; }

    if (hasCustomCss) console.log('✅ docusaurus.config.js - References custom CSS');
    else { console.log('⚠️  docusaurus.config.js - No custom CSS reference found'); }
  } else {
    console.log('❌ docusaurus.config.js - File does not exist');
    allGood = false;
  }
} catch (error) {
  console.log(`❌ Error reading docusaurus.config.js: ${error.message}`);
  allGood = false;
}

console.log('\n📋 Checking CSS file...\n');

try {
  const cssPath = path.join(__dirname, 'src', 'css', 'custom.css');
  if (fs.existsSync(cssPath)) {
    const cssContent = fs.readFileSync(cssPath, 'utf8');
    const hasColorVars = cssContent.includes('--ifm-color-primary');
    const hasRootVars = cssContent.includes(':root');
    const hasDarkMode = cssContent.includes('[data-theme=') || cssContent.includes('dark');

    if (hasColorVars) console.log('✅ custom.css - Contains color variables');
    else { console.log('⚠️  custom.css - No color variables found'); }

    if (hasRootVars) console.log('✅ custom.css - Contains root variables');
    else { console.log('⚠️  custom.css - No root variables found'); }

    if (hasDarkMode) console.log('✅ custom.css - Contains dark mode support');
    else { console.log('⚠️  custom.css - No dark mode support found'); }
  } else {
    console.log('❌ custom.css - File does not exist');
    allGood = false;
  }
} catch (error) {
  console.log(`❌ Error reading custom.css: ${error.message}`);
  allGood = false;
}

console.log('\n📋 Checking documentation structure...\n');

try {
  const docsPath = path.join(__dirname, 'docs');
  if (fs.existsSync(docsPath) && fs.statSync(docsPath).isDirectory()) {
    const docFiles = fs.readdirSync(docsPath).filter(f => f.endsWith('.md') || f.endsWith('.mdx'));
    console.log(`✅ docs/ - Directory exists with ${docFiles.length} documentation files`);

    if (docFiles.includes('intro.md')) {
      console.log('✅ intro.md - Found in docs directory');
    } else {
      console.log('❌ intro.md - Not found in docs directory');
      allGood = false;
    }
  } else {
    console.log('❌ docs/ - Directory does not exist');
    allGood = false;
  }
} catch (error) {
  console.log(`❌ Error checking docs directory: ${error.message}`);
  allGood = false;
}

console.log('\n🎯 Validation results:');
if (allGood) {
  console.log('✅ Environment is properly set up for Docusaurus UI fixes!');
  console.log('You can proceed with the UI implementation tasks.');
  process.exit(0);
} else {
  console.log('❌ Some issues were found. Please resolve them before proceeding.');
  process.exit(1);
}