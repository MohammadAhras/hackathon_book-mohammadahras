# Quickstart: Digital Twin Simulation Development

## Prerequisites

- Node.js v18.0 or higher
- npm or yarn package manager
- Git for version control
- Basic knowledge of Markdown and JavaScript
- Familiarity with simulation concepts (Gazebo, Unity) preferred but not required

## Setup Process

### 1. Verify Docusaurus Installation
The Docusaurus project should already be set up from the previous ROS 2 module. Verify the project structure:
```bash
npm run start
```

### 2. Project Structure Setup
The project structure should include:
```
my-website/
├── docs/                 # Documentation files
│   ├── intro.md
│   ├── ros2-fundamentals/    # Existing ROS 2 module
│   ├── python-agents/        # Existing ROS 2 module
│   ├── humanoid-modeling/    # Existing ROS 2 module
│   └── digital-twin-simulation/  # New module directory
├── src/
│   ├── components/   # Custom React components
│   ├── css/          # Custom styles
│   └── pages/        # Custom pages
├── static/       # Static files
├── docusaurus.config.js  # Site configuration
├── package.json
└── sidebars.js   # Navigation configuration
```

### 3. Configure the Site
Update `docusaurus.config.js` with your project details if needed:
- Site title and tagline (should already be configured)
- Organization name and project name
- URL and base URL for deployment
- Theme and plugin configurations

### 4. Organize Content
Create the following structure in the `docs/digital-twin-simulation/` directory:
```
docs/digital-twin-simulation/
├── index.md
├── physics-simulation-gazebo/
│   ├── index.md
│   ├── gravity-collisions-dynamics.md
│   ├── world-robot-simulation.md
│   └── role-gazebo-robotics-testing.md
├── high-fidelity-unity/
│   ├── index.md
│   ├── visual-realism-interaction.md
│   ├── human-robot-interaction-scenarios.md
│   └── unity-gazebo-integration.md
├── sensor-simulation/
│   ├── index.md
│   ├── lidar-depth-cameras-imus.md
│   ├── sensor-noise-realism.md
│   └── simulation-reality-considerations.md
└── _category_.json
```

### 5. Create Category Files
Create `_category_.json` files to define navigation:

**docs/digital-twin-simulation/_category_.json:**
```json
{
  "label": "Digital Twin Simulation",
  "position": 4,
  "link": {
    "type": "generated-index",
    "description": "Learn physics-based simulation and digital twin creation for humanoid robots using Gazebo and Unity."
  }
}
```

### 6. Update Navigation
Update `sidebars.js` to include the new module in the navigation structure.

### 7. Run Development Server
```bash
npm run start
```
This command starts a local development server and opens your site in a browser. Most changes are reflected live without restarting the server.

### 8. Build for Production
```bash
npm run build
```
This command generates static content into the `build/` directory, which can be served using any static hosting service.

### 9. Validate Navigation
Test all internal links and navigation to ensure users can move seamlessly between chapters and sections.