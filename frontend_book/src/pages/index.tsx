import React from 'react';
import Layout from '@theme/Layout';
import HeroSection from '../components/HeroSection';

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
                <a
                  className="feature-card depth-1"
                  href={card.link}
                  style={{
                    display: 'block',
                    padding: '2rem',
                    borderRadius: '12px',
                    background: 'var(--futuristic-bg-dark)',
                    border: '1px solid var(--futuristic-border-color)',
                    transition: 'all 0.3s ease',
                    textDecoration: 'none',
                    color: 'inherit',
                    position: 'relative',
                    overflow: 'hidden',
                    boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)'
                  }}
                >
                  <div style={{fontSize: '2rem', marginBottom: '1rem'}}>{card.icon}</div>
                  <h3
                    style={{
                      fontSize: '1.25rem',
                      fontWeight: '600',
                      margin: '0 0 0.5rem 0',
                      color: 'var(--futuristic-text-primary)',
                      transition: 'color 0.3s ease'
                    }}
                  >
                    {card.title}
                  </h3>
                  <p
                    style={{
                      color: 'var(--futuristic-text-secondary)',
                      margin: '0 0 1rem 0',
                      lineHeight: '1.6'
                    }}
                  >
                    {card.description}
                  </p>
                  <div
                    style={{
                      color: 'var(--ifm-color-primary)',
                      fontWeight: 'bold',
                      opacity: 0,
                      transform: 'translateX(-10px)',
                      transition: 'all 0.3s ease'
                    }}
                    className="card-arrow"
                  >
                    →
                  </div>
                </a>
              </div>
            ))}
          </div>
        </div>
      </main>
    </Layout>
  );
}
