# Quickstart Guide: Docusaurus UI Upgrade

## Overview

This guide provides a quick path to implement the modern, futuristic UI upgrade for the Docusaurus documentation site. Follow these steps to transform the existing basic design into a cinematic, AI/humanoid-themed interface with dark theme, gradients, and glow effects.

## Prerequisites

- Node.js >= 18.0
- npm or yarn package manager
- Git for version control
- Basic knowledge of React and CSS
- Docusaurus v3.9.2 (already installed)

## Setup Environment

1. **Navigate to the frontend_book directory:**
   ```bash
   cd frontend_book
   ```

2. **Install dependencies (if not already installed):**
   ```bash
   npm install
   ```

3. **Start the development server:**
   ```bash
   npm run start
   ```

## Implementation Steps

### 1. Create UI Components

Create the following component files in `src/components/`:

#### HeroSection Component
```bash
# Create the components directory if it doesn't exist
mkdir -p src/components
```

Create `src/components/HeroSection.tsx`:
```tsx
import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './HeroSection.css';

interface HeroSectionProps {
  title?: string;
  subtitle?: string;
  ctaText?: string;
  ctaLink?: string;
}

const HeroSection: React.FC<HeroSectionProps> = ({
  title = "Physical AI & Humanoid Robotics",
  subtitle = "Modern documentation for ROS 2, URDF, and Robotics",
  ctaText = "Start Reading",
  ctaLink = "/docs/intro"
}) => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <section className="hero-section">
      <div className="hero-container">
        <h1 className="hero-title">{title}</h1>
        <p className="hero-subtitle">{subtitle}</p>
        <div className="hero-cta">
          <Link className="cta-button button button--primary button--lg" to={ctaLink}>
            {ctaText} →
          </Link>
        </div>
      </div>
    </section>
  );
};

export default HeroSection;
```

Create `src/components/HeroSection.css`:
```css
.hero-section {
  padding: 4rem 1rem;
  text-align: center;
  position: relative;
  overflow: hidden;
  background: linear-gradient(135deg, var(--futuristic-bg-darker) 0%, var(--futuristic-bg-dark) 100%);
}

.hero-section::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: radial-gradient(circle at top right, rgba(37, 194, 160, 0.1) 0%, transparent 40%),
              radial-gradient(circle at bottom left, rgba(30, 64, 175, 0.1) 0%, transparent 40%);
  pointer-events: none;
}

.hero-container {
  position: relative;
  z-index: 1;
  max-width: var(--ifm-container-width);
  margin: 0 auto;
}

.hero-title {
  font-size: 3rem;
  font-weight: 700;
  margin-bottom: 1rem;
  background: linear-gradient(to right, var(--ifm-color-primary) 0%, var(--futuristic-accent) 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  text-fill-color: transparent;
  text-shadow: 0 0 20px rgba(37, 194, 160, 0.3);
}

.hero-subtitle {
  font-size: 1.25rem;
  color: var(--futuristic-text-secondary);
  margin-bottom: 2rem;
  max-width: 600px;
  margin-left: auto;
  margin-right: auto;
}

.hero-cta {
  margin-top: 2rem;
}

.cta-button {
  background: linear-gradient(135deg, var(--ifm-color-primary) 0%, var(--futuristic-accent) 100%);
  border: none;
  padding: 1rem 2rem;
  font-size: 1.1rem;
  font-weight: 600;
  border-radius: 8px;
  box-shadow: 0 4px 20px rgba(37, 194, 160, 0.3);
  transition: all 0.3s ease;
}

.cta-button:hover {
  transform: translateY(-2px);
  box-shadow: 0 8px 30px rgba(37, 194, 160, 0.4);
  text-decoration: none;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2rem;
  }

  .hero-subtitle {
    font-size: 1rem;
  }
}
```

#### FeatureCard Component
Create `src/components/FeatureCard.tsx`:
```tsx
import React from 'react';
import Link from '@docusaurus/Link';
import './FeatureCard.css';

interface FeatureCardProps {
  title: string;
  description: string;
  link: string;
  icon?: string;
  category?: string;
  depth?: number;
}

const FeatureCard: React.FC<FeatureCardProps> = ({
  title,
  description,
  link,
  icon = '📄',
  depth = 1
}) => {
  return (
    <Link className={`feature-card depth-${depth}`} to={link}>
      <div className="card-icon">{icon}</div>
      <h3 className="card-title">{title}</h3>
      <p className="card-description">{description}</p>
      <div className="card-arrow">→</div>
    </Link>
  );
};

export default FeatureCard;
```

Create `src/components/FeatureCard.css`:
```css
.feature-card {
  display: block;
  padding: 2rem;
  border-radius: 12px;
  background: var(--futuristic-bg-dark);
  border: 1px solid var(--futuristic-border-color);
  transition: all 0.3s ease;
  text-decoration: none;
  color: inherit;
  position: relative;
  overflow: hidden;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.feature-card::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: var(--futuristic-gradient-card);
  opacity: 0;
  transition: opacity 0.3s ease;
  z-index: -1;
}

.feature-card:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2);
  border-color: var(--futuristic-accent);
}

.feature-card:hover::before {
  opacity: 0.1;
}

.feature-card:hover .card-title {
  color: var(--ifm-color-primary);
}

.depth-1 {
  border-left: 4px solid var(--ifm-color-primary);
}

.depth-2 {
  border-left: 4px solid var(--futuristic-accent);
}

.depth-3 {
  border-left: 4px solid var(--futuristic-secondary);
}

.card-icon {
  font-size: 2rem;
  margin-bottom: 1rem;
}

.card-title {
  font-size: 1.25rem;
  font-weight: 600;
  margin: 0 0 0.5rem 0;
  color: var(--futuristic-text-primary);
  transition: color 0.3s ease;
}

.card-description {
  color: var(--futuristic-text-secondary);
  margin: 0 0 1rem 0;
  line-height: 1.6;
}

.card-arrow {
  color: var(--ifm-color-primary);
  font-weight: bold;
  opacity: 0;
  transform: translateX(-10px);
  transition: all 0.3s ease;
}

.feature-card:hover .card-arrow {
  opacity: 1;
  transform: translateX(0);
}

@media (max-width: 768px) {
  .feature-card {
    padding: 1.5rem;
  }

  .card-title {
    font-size: 1.1rem;
  }
}
```

### 2. Update CSS Variables

Update `src/css/custom.css` to include the new theme variables:

```css
/* Futuristic theme variables */
:root {
  /* Existing variables preserved */
  --ifm-color-primary: #25c2a0;
  --ifm-color-primary-dark: #1fa588;
  --ifm-color-primary-darker: #1b9077;
  --ifm-color-primary-darkest: #177d69;
  --ifm-color-primary-light: #46d3b1;
  --ifm-color-primary-lighter: #6be0c3;
  --ifm-color-primary-lightest: #90edda;
  --ifm-color-primary-blue: #1e40af;
  --ifm-color-primary-blue-dark: #1d4ed8;
  --ifm-color-primary-blue-darker: #1e3a8a;
  --ifm-color-primary-blue-darkest: #1e3c72;
  --ifm-color-primary-blue-light: #3b82f6;
  --ifm-color-primary-blue-lighter: #60a5fa;
  --ifm-color-primary-blue-lightest: #93c5fd;
  --ifm-code-font-size: 95%;
  --ifm-spacing-vertical: 1.5rem;
  --ifm-spacing-horizontal: 1.5rem;
  --ifm-spacing-scale: 1.25rem;
  --ifm-color-text: #1a1a1a;
  --ifm-color-text-light: #333333;
  --ifm-color-text-lighter: #4d4d4d;
  --ifm-color-text-lightest: #666666;
  --ifm-color-text-dark: #000000;

  /* New futuristic theme variables */
  --futuristic-primary: #25c2a0;
  --futuristic-secondary: #1e40af;
  --futuristic-accent: #60a5fa;
  --futuristic-bg-dark: #0f172a;
  --futuristic-bg-darker: #020617;
  --futuristic-text-primary: #f1f5f9;
  --futuristic-text-secondary: #cbd5e1;
  --futuristic-border-color: #334155;
  --futuristic-gradient-main: linear-gradient(135deg, #1e40af 0%, #25c2a0 100%);
  --futuristic-gradient-card: linear-gradient(135deg, #1e40af 0%, #25c2a0 100%);
  --futuristic-gradient-button: linear-gradient(135deg, #25c2a0 0%, #60a5fa 100%);
  --futuristic-glow-primary: 0 0 15px rgba(37, 194, 160, 0.5);
  --futuristic-glow-secondary: 0 0 15px rgba(96, 165, 250, 0.5);
  --futuristic-glow-intensity: 0.3;
}

/* Dark theme overrides */
[data-theme='dark'] {
  --futuristic-primary: #25c2a0;
  --futuristic-secondary: #3b82f6;
  --futuristic-accent: #60a5fa;
  --futuristic-bg-dark: #0f172a;
  --futuristic-bg-darker: #020617;
  --futuristic-text-primary: #f1f5f9;
  --futuristic-text-secondary: #cbd5e1;
  --futuristic-border-color: #334155;
}
```

### 3. Update Homepage

Replace `src/pages/index.tsx` with the new modern layout:

```tsx
import React from 'react';
import Layout from '@theme/Layout';
import HeroSection from '../components/HeroSection';
import FeatureCard from '../components/FeatureCard';

export default function Home() {
  // Sample feature cards data - replace with actual documentation sections
  const featureCards = [
    {
      title: "ROS 2 Fundamentals",
      description: "Learn the basics of ROS 2, the next generation robotics framework",
      link: "/docs/ros2-fundamentals/intro",
      icon: "🤖"
    },
    {
      title: "Humanoid Modeling",
      description: "Create and simulate humanoid robots using URDF and advanced kinematics",
      link: "/docs/humanoid-modeling/intro",
      icon: "🦾"
    },
    {
      title: "Digital Twin Simulation",
      description: "Build realistic simulations with Gazebo and advanced physics engines",
      link: "/docs/digital-twin-simulation/intro",
      icon: "🎮"
    },
    {
      title: "AI-Robot Brain (Isaac)",
      description: "Implement intelligent behaviors with NVIDIA Isaac robotics platform",
      link: "/docs/ai-robot-brain-isaac",
      icon: "🧠"
    },
    {
      title: "Vision-Language-Action",
      description: "Combine perception, language, and control for advanced robot behaviors",
      link: "/docs/vla-module",
      icon: "👁️"
    },
    {
      title: "Docusaurus UI Upgrade",
      description: "Learn about the modern UI implementation and design principles",
      link: "/docs/docusaurus-ui",
      icon: "🎨"
    }
  ];

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Modern ROS 2 documentation with futuristic UI">
      <main>
        <HeroSection />

        <div className="container padding-horiz--md">
          <div className="row">
            <div className="col col--12">
              <h2 style={{textAlign: 'center', marginTop: '3rem', marginBottom: '2rem'}}>
                Explore Documentation Sections
              </h2>
            </div>
          </div>

          <div className="row" style={{gap: '2rem', justifyContent: 'center'}}>
            {featureCards.map((card, index) => (
              <div key={index} className="col col--12 col--md-4">
                <FeatureCard
                  title={card.title}
                  description={card.description}
                  link={card.link}
                  icon={card.icon}
                  depth={index % 3 + 1}
                />
              </div>
            ))}
          </div>
        </div>
      </main>
    </Layout>
  );
}
```

### 4. Build and Test

1. **Stop the development server** (Ctrl+C)

2. **Build the site:**
   ```bash
   npm run build
   ```

3. **Serve the build locally:**
   ```bash
   npm run serve
   ```

4. **Open your browser to** `http://localhost:3000` to see the new UI

### 5. Additional Customizations

#### Add AI/Humanoid Visual Elements

You can add visual elements to the HeroSection by updating the HeroSection.css to include background patterns or images:

```css
/* Add to HeroSection.css after the ::before pseudo-element */
.hero-section {
  /* ... existing styles ... */
  background-image:
    radial-gradient(circle at 10% 20%, rgba(30, 64, 175, 0.1) 0%, transparent 20%),
    radial-gradient(circle at 90% 80%, rgba(37, 194, 160, 0.1) 0%, transparent 20%),
    linear-gradient(135deg, var(--futuristic-bg-darker) 0%, var(--futuristic-bg-dark) 100%);
}
```

## Deployment

When ready for production:

1. **Build the site:**
   ```bash
   npm run build
   ```

2. **The build output will be in the `build/` directory**

3. **Deploy to your preferred hosting platform (Vercel, GitHub Pages, etc.)**

## Troubleshooting

- **Components not rendering**: Ensure all component files are saved and the development server has restarted
- **Styles not applying**: Check that custom.css is properly imported in docusaurus.config.js
- **Links not working**: Verify that all documentation routes exist in sidebars.js
- **Dark mode issues**: Check that all theme variables have proper dark mode overrides