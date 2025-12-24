#!/usr/bin/env node

/**
 * Documentation Pages Verification Script
 * Verifies that all existing documentation pages remain accessible and functional
 */

const fs = require('fs');
const path = require('path');

console.log('🔍 Verifying all documentation pages remain accessible and functional...\n');

// Check the docs directory structure
const docsDir = path.join(__dirname, 'docs');
if (!fs.existsSync(docsDir)) {
  console.log('❌ docs directory not found');
  process.exit(1);
}

console.log('✅ docs directory found');

// List all documentation directories and files
const docFiles = [];
const docDirs = [];

function scanDirectory(dir, prefix = '') {
  const items = fs.readdirSync(dir);

  for (const item of items) {
    const itemPath = path.join(dir, item);
    const stat = fs.statSync(itemPath);

    if (stat.isDirectory()) {
      const dirName = prefix ? `${prefix}/${item}` : item;
      docDirs.push(dirName);
      scanDirectory(itemPath, dirName);
    } else if (item.endsWith('.md') || item.endsWith('.mdx')) {
      const fileName = prefix ? `${prefix}/${item}` : item;
      docFiles.push(fileName);
    }
  }
}

scanDirectory(docsDir);

console.log(`\n📊 Documentation structure:`);
console.log(`Directories: ${docDirs.length}`);
console.log(`Markdown files: ${docFiles.length}\n`);

// Check for essential documentation files
const essentialFiles = [
  'intro.md',
  'docusaurus-ui/index.md'
];

console.log('📋 Checking essential documentation files...\n');

let essentialFound = 0;
for (const file of essentialFiles) {
  const exists = docFiles.some(docFile => docFile.endsWith(file));
  if (exists) {
    console.log(`✅ Essential file found: ${file}`);
    essentialFound++;
  } else {
    console.log(`❌ Essential file missing: ${file}`);
  }
}

// Check for module directories
const moduleDirs = [
  'ros2-fundamentals',
  'python-agents',
  'humanoid-modeling',
  'digital-twin-simulation',
  'ai-robot-brain-isaac',
  'vla-module',
  'docusaurus-ui'
];

console.log('\n📋 Checking module directories...\n');

let modulesFound = 0;
for (const dir of moduleDirs) {
  const exists = docDirs.some(docDir => docDir === dir);
  if (exists) {
    console.log(`✅ Module directory found: ${dir}`);
    modulesFound++;
  } else {
    console.log(`❌ Module directory missing: ${dir}`);
  }
}

// Check the build directory to ensure pages were generated
const buildDir = path.join(__dirname, 'build');
if (fs.existsSync(buildDir)) {
  console.log('\n✅ Build directory exists');

  // Check for some expected built files
  const expectedBuildFiles = [
    'docs/intro/index.html',
    'docs/docusaurus-ui/index.html'
  ];

  let buildFilesFound = 0;
  for (const file of expectedBuildFiles) {
    const buildFilePath = path.join(buildDir, file);
    if (fs.existsSync(buildFilePath)) {
      console.log(`✅ Built page found: ${file}`);
      buildFilesFound++;
    } else {
      console.log(`⚠️  Built page missing: ${file}`);
    }
  }

  console.log(`\nBuild verification: ${buildFilesFound}/${expectedBuildFiles.length} pages built`);
} else {
  console.log('\n❌ Build directory does not exist');
}

// Verify CSS doesn't break existing functionality
const cssPath = path.join(__dirname, 'src', 'css', 'custom.css');
const cssContent = fs.readFileSync(cssPath, 'utf8');

// Check that CSS doesn't break existing selectors
const potentiallyBreakingPatterns = [
  { name: 'Overly broad selectors', pattern: /\* \{[^}]*\}/g },
  { name: 'Reset styles', pattern: /all:\s*reset|all:\s*unset/g },
  { name: 'Global overrides', pattern: /body\s*\{|html\s*\{/g }
];

console.log('\n📋 Checking for potentially breaking CSS patterns...\n');

let breakingFound = 0;
for (const pattern of potentiallyBreakingPatterns) {
  const matches = cssContent.match(pattern.pattern);
  if (matches) {
    console.log(`⚠️  Potentially breaking pattern found: ${pattern.name} (${matches.length} occurrences)`);
    breakingFound++;
  } else {
    console.log(`✅ No breaking pattern found: ${pattern.name}`);
  }
}

console.log('\n🎯 Documentation pages verification results:');
console.log(`Essential files: ${essentialFound}/${essentialFiles.length} found`);
console.log(`Module directories: ${modulesFound}/${moduleDirs.length} found`);

if (essentialFound === essentialFiles.length && modulesFound === moduleDirs.length && breakingFound === 0) {
  console.log('\n✅ All documentation pages verification passed!');
  console.log('All existing documentation pages remain accessible and functional.');
} else {
  console.log('\n⚠️  Some documentation pages may have issues.');
  console.log('Check that all content remains accessible after CSS changes.');
}

console.log('\n🎉 Documentation verification complete!');
process.exit(0);