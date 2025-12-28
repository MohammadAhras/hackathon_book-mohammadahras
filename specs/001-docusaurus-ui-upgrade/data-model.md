# Data Model: Docusaurus UI Upgrade

## Overview

This data model describes the UI components and their properties for the Docusaurus UI upgrade. Since this is primarily a visual enhancement feature, the "data" consists of component properties and styling attributes rather than traditional data entities.

## Component Models

### 1. HeroSection Component

**Purpose**: Cinematic hero section with bold typography and AI/humanoid visuals

**Properties**:
- `title`: string - Main headline text
- `subtitle`: string - Supporting text
- `ctaText`: string - Call-to-action button text (default: "Start Reading")
- `ctaLink`: string - Destination link for CTA button
- `visualElements`: array - Visual elements to include (AI/humanoid imagery)
- `theme`: string - Theme variant ("dark", "light", "cinematic")

**States**:
- `loading`: boolean - Whether content is still loading
- `error`: string | null - Error message if content fails to load

### 2. FeatureCard Component

**Purpose**: Modern cards for displaying documentation modules/sections

**Properties**:
- `title`: string - Card title
- `description`: string - Brief description of the section
- `icon`: string - Optional icon identifier
- `link`: string - Link to the documentation section
- `category`: string - Category or section identifier
- `isNew`: boolean - Whether this is a new section

**Visual Properties**:
- `gradient`: string - CSS gradient definition for background
- `glowEffect`: boolean - Whether to apply glow effect
- `depth`: number - Visual depth level (1-3)

### 3. CTAButton Component

**Purpose**: Prominent call-to-action button

**Properties**:
- `text`: string - Button text
- `link`: string - Destination URL
- `variant`: string - Button style ("primary", "secondary", "ghost")
- `size`: string - Button size ("small", "medium", "large")
- `isLoading`: boolean - Whether button is in loading state

**Visual Properties**:
- `glow`: boolean - Whether to apply glow effect
- `gradient`: string - CSS gradient for button background

### 4. ThemeProvider Component

**Purpose**: Manages dark theme with gradients and glow effects

**Properties**:
- `theme`: string - Current theme ("dark", "light")
- `primaryColor`: string - Primary color in hex/RGB format
- `secondaryColor`: string - Secondary color in hex/RGB format
- `gradientPresets`: array - Available gradient configurations

**State**:
- `isDarkMode`: boolean - Whether dark mode is active
- `themeConfig`: object - Theme configuration object

## Style Variables

### Color Variables
- `--futuristic-primary`: Primary color for the theme
- `--futuristic-secondary`: Secondary color
- `--futuristic-accent`: Accent color for highlights
- `--futuristic-bg-dark`: Dark background color
- `--futuristic-bg-darker`: Darker background color
- `--futuristic-text-primary`: Primary text color
- `--futuristic-text-secondary`: Secondary text color

### Gradient Variables
- `--futuristic-gradient-main`: Main gradient for backgrounds
- `--futuristic-gradient-card`: Gradient for cards
- `--futuristic-gradient-button`: Gradient for buttons

### Glow Effects
- `--futuristic-glow-primary`: Primary glow effect
- `--futuristic-glow-secondary`: Secondary glow effect
- `--futuristic-glow-intensity`: Glow intensity level

### Typography Variables
- `--futuristic-font-family`: Primary font family
- `--futuristic-font-size-h1`: H1 font size
- `--futuristic-font-size-h2`: H2 font size
- `--futuristic-font-weight-bold`: Bold font weight
- `--futuristic-line-height`: Line height for readability

## Layout Components

### 1. GridLayout Component

**Purpose**: Responsive grid layout for feature cards

**Properties**:
- `columns`: number - Number of columns (responsive)
- `gap`: string - Gap between items
- `align`: string - Alignment of items ("start", "center", "end")

### 2. Container Component

**Purpose**: Main content container with proper spacing

**Properties**:
- `maxWidth`: string - Maximum width of container
- `padding`: string - Padding around content
- `centered`: boolean - Whether content should be centered

## Validation Rules

### Component Validation
- All components must accept a `className` prop for custom styling
- All links must be valid URLs or internal routes
- All text content must support internationalization
- All interactive elements must support keyboard navigation

### Accessibility Validation
- All components must meet WCAG 2.1 AA contrast ratios
- Interactive elements must have proper ARIA labels
- Focus indicators must be visible
- Semantic HTML elements must be used appropriately

## State Transitions

### Theme Switching
- `light` → `dark`: Smooth transition with CSS transitions
- `dark` → `light`: Smooth transition with CSS transitions

### Loading States
- `idle` → `loading`: Show loading indicators
- `loading` → `loaded`: Display content
- `loading` → `error`: Display error state