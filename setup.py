from setuptools import find_packages, setup


package_name = "ss_depth"


setup(
  name=package_name,
  version="0.1.0",
  packages=find_packages(exclude=["test"]),
  data_files=[
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml", "README.md"]),
    (f"share/{package_name}/launch", ["launch/dashboard.launch.py"]),
    (
      f"share/{package_name}/scripts",
      [
        "scripts/start_dashboard_terminals.sh",
        "scripts/stop_dashboard_terminals.sh",
        "scripts/terminal_runner.sh",
      ],
    ),
  ],
  install_requires=["setuptools"],
  zip_safe=True,
  maintainer="saeumsoft",
  maintainer_email="saeumsoft@example.com",
  description="ROS2 dashboard package for obstacle avoidance visualization.",
  license="Apache-2.0",
  tests_require=["pytest"],
  entry_points={
    "console_scripts": [
      "dashboard_node = ss_depth.dashboard_node:main",
    ],
  },
)
