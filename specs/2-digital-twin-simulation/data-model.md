# Data Model: Digital Twin Simulation

## Content Organization Model

### Chapter Structure
The educational module is organized into three main chapters, each containing multiple articles:

```
Chapter {
  id: string,
  title: string,
  description: string,
  articles: [Article],
  position: number
}
```

### Article Structure
Each article in the module follows a consistent format:

```
Article {
  id: string,
  title: string,
  sidebar_label: string,
  description: string,
  tags: [string],
  keywords: [string],
  slug: string
}
```

## Docusaurus Configuration Model

### Site Configuration
The main site configuration in `docusaurus.config.js` includes:

```
SiteConfig {
  title: string,
  tagline: string,
  url: string,
  baseUrl: string,
  organizationName: string,
  projectName: string,
  onBrokenLinks: "throw" | "warn" | "ignore",
  onBrokenMarkdownLinks: "warn" | "ignore",
  favicon: string,
  themes: [string],
  presets: [PresetConfig]
}
```

### Sidebar Model
Navigation is defined in `sidebars.js` with:

```
Sidebar {
  tutorial: [
    {
      type: "category",
      label: string,
      items: [string | SidebarItem],
      collapsed: boolean
    }
  ]
}
```

## Content Metadata Model

### Frontmatter Schema
Each markdown file includes frontmatter with:

```
Frontmatter {
  id?: string,
  title: string,
  sidebar_label?: string,
  description?: string,
  tags?: [string],
  keywords?: [string],
  slug?: string,
  hide_table_of_contents?: boolean
}
```

## Educational Content Model

### Learning Objectives
Each chapter and article defines clear learning objectives:

```
LearningObjective {
  id: string,
  text: string,
  level: "understand" | "apply" | "analyze" | "create",
  related_articles: [string]
}
```

### Simulation Content Model
Technical content includes simulation examples:

```
SimulationExample {
  title: string,
  description: string,
  code: string,
  language: "xml" | "python" | "javascript" | "bash",
  simulation_environment: "gazebo" | "unity" | "both"
}
```

## Navigation Model

### Breadcrumb Structure
The site provides clear navigation paths:

```
Breadcrumb {
  current: string,
  parent: string,
  siblings: [string],
  next: string,
  previous: string
}
```

### Cross-Reference Model
Articles reference related content:

```
CrossReference {
  type: "see_also" | "prerequisite" | "next" | "related_concept",
  target_article: string,
  relationship: string
}
```

## Simulation Concepts Model

### Physics Simulation Concepts
Core concepts for Gazebo-based physics simulation:

```
PhysicsSimulationConcept {
  id: string,
  name: string,
  description: string,
  parameters: [Parameter],
  applications: [string]
}

Parameter {
  name: string,
  type: "float" | "int" | "bool" | "string",
  default_value: any,
  description: string,
  range?: [number, number]
}
```

### Sensor Simulation Model
Models for different sensor types:

```
SensorModel {
  type: "lidar" | "camera" | "imu" | "gps" | "force_torque",
  name: string,
  properties: [Property],
  noise_model: NoiseModel
}

NoiseModel {
  type: "gaussian" | "uniform" | "custom",
  parameters: [Parameter]
}
```

### Unity Integration Model
Concepts for Unity-based high-fidelity environments:

```
UnityEnvironment {
  name: string,
  complexity_level: "basic" | "intermediate" | "advanced",
  features: [string],
  human_robot_interaction_scenarios: [string]
}
```