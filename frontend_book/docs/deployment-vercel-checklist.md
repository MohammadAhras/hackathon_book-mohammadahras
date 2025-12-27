# Vercel Deployment Checklist for Docusaurus Site

## Pre-Deployment Checklist

- [ ] **Repository Ready**: Code is committed and pushed to GitHub repository
- [ ] **Build Verification**: `npm run build` completes successfully locally
- [ ] **Configuration Review**:
  - [ ] `url` in docusaurus.config.js matches expected Vercel domain
  - [ ] `baseUrl` set to '/' for root deployment
  - [ ] `routeBasePath` set to 'docs' for proper documentation routing
  - [ ] All links in footer and navigation are correct
- [ ] **Documentation Paths**: Verify all document paths exist and are accessible
- [ ] **Custom CSS**: All custom styling preserved after build

## Vercel Dashboard Setup

- [ ] **Import Repository**: Connect GitHub repository to Vercel account
- [ ] **Project Name**: Set appropriate project name (e.g., hackathon-book-mohammadahras)
- [ ] **Build Settings**:
  - [ ] Framework Preset: Auto or select "Docusaurus"
  - [ ] Build Command: `npm run build`
  - [ ] Output Directory: `build`
  - [ ] Root Directory: `/` (root of repository)
- [ ] **Environment Variables**: None required for basic Docusaurus site

## Configuration Verification

- [ ] **Root Path**: Site accessible at https://[project-name].vercel.app/
- [ ] **Documentation Access**: Docs accessible at https://[project-name].vercel.app/docs/
- [ ] **Navigation Links**: All internal links function properly
- [ ] **External Links**: All external links (GitHub, etc.) work correctly
- [ ] **Assets**: Images, CSS, and JS files load properly
- [ ] **Responsive Design**: Site displays correctly on mobile and desktop

## Post-Deployment Verification

- [ ] **No 404 Errors**: All pages load without resource errors
- [ ] **Base URL**: No "Wrong baseUrl configuration" warnings
- [ ] **Search Functionality**: If enabled, search works properly
- [ ] **Versioning**: If applicable, versioned docs are accessible
- [ ] **Analytics**: If configured, analytics track properly

## Troubleshooting Checklist

- [ ] **baseUrl Issue**: If seeing baseUrl warnings, verify `baseUrl: '/'` in config
- [ ] **Broken Links**: If links broken, check that `routeBasePath` matches actual paths
- [ ] **Asset Loading**: If CSS/images not loading, verify paths in config
- [ ] **Build Failures**: Check Vercel logs for specific error messages
- [ ] **Redirect Issues**: Verify vercel.json has proper catch-all routing if needed