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
        <div className="hero-visual-elements">
          <div className="ai-element">🤖</div>
          <div className="humanoid-element">🦾</div>
        </div>
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