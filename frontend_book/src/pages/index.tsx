import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Modern ROS 2 documentation">
      <main style={{padding: '4rem', textAlign: 'center'}}>
        <h1>ROS 2 Robotics Module</h1>
        <p>Modern documentation for ROS 2, URDF, and Robotics</p>

        <Link
          className="button button--primary button--lg"
          to="/docs/intro">
          Get Started 🚀
        </Link>
      </main>
    </Layout>
  );
}
