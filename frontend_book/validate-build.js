#!/usr/bin/env node

/**
 * Build Validation Script
 * Validates that the Docusaurus site builds successfully after UI changes
 */

const { exec } = require('child_process');
const path = require('path');

console.log('🏗️  Validating Docusaurus build...\n');

// Change to the frontend_book directory
const projectDir = path.join(__dirname);

// Run the build command
const buildProcess = exec('npm run build', { cwd: projectDir }, (error, stdout, stderr) => {
  if (error) {
    console.log('❌ Build failed!');
    console.log(`Error: ${error.message}`);
    console.log(`Exit code: ${error.code}`);
    process.exit(1);
  }

  if (stderr) {
    console.log('⚠️  Build completed with warnings:');
    console.log(stderr);
  }

  console.log('✅ Build completed successfully!');
  console.log('The Docusaurus site builds without errors.');

  // Check if the build directory was created
  const fs = require('fs');
  const buildDir = path.join(projectDir, 'build');
  if (fs.existsSync(buildDir) && fs.statSync(buildDir).isDirectory()) {
    console.log('✅ Build directory created successfully.');
  } else {
    console.log('⚠️  Build directory not found - this may be unexpected.');
  }

  console.log('\n🎉 Build validation passed! The UI changes are compatible with the build process.');
  process.exit(0);
});

// Show build progress
buildProcess.stdout.on('data', (data) => {
  process.stdout.write(data);
});

buildProcess.stderr.on('data', (data) => {
  process.stderr.write(data);
});