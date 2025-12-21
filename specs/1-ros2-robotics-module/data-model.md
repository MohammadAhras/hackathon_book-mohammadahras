# Data Model: ROS 2 Robotics Module

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

### Code Example Model
Technical content includes executable code examples:

```
CodeExample {
  title: string,
  description: string,
  code: string,
  language: "python" | "xml" | "bash" | "javascript",
  execution_context: "simulation" | "real_robot" | "theoretical"
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