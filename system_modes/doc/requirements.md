# Requirements

We discussed our first analysis results with the architect and developers of the above use-case as well as experts for robotics systems and software engineering and derived the following general requirements for the intended concepts and abstractions for (1.) system runtime configuration and (2.) system error and contingency diagnosis.

## System Runtime Configuration

The envisioned approach shall provide concepts for modeling of the system hierarchy with regard to system configuration / modes as well as corresponding framework functionalities.

1.  Model
    *   **Hierarchical modeling of subsystems**
        *   The envisioned approach shall allow to define subsystems consisting of multiple components (ROS nodes, micro-ROS nodes) in a hierarchical or at least two-staged (component - sub-system - system) manner.
        *   TODO: Are such hierarchies also relevant for diagnosis/monitoring? If yes, explore whether the same hierarchy should be used.
        *   Some robotic frameworks feature meta-components or hierarchical components. We prefer a more flexible approach allowing to define subsystems for each aspect (system modes, scheduling, distribution, ...) individually. This approach gives more flexibility - e.g., to treat all device drivers in a uniform manner with regard to scheduling but put the camera driver and the feature and obstacle recognition into one sub-system with regard to system configuration. Also, it allows the application developer to only use mechanisms and framework features that are actually relevant for his system - e.g., distribution might be irrelevant for small robotic systems.
    *   **Modeling of system, sub-system and component modes**
        *   Based on the definition the system, sub-systems and components, the envisioned system runtime configuration concept shall provide an approach for defining individual modes of each of these elements and specify the interconnection between them.
    *   **Standard but extensible component runtime lifecycle**
        *   The component runtime lifecycle (component mode/state) model shall be based on the OMG standard used in [ROS 2](http://design.ros2.org/articles/node_lifecycle.html) and which also resembles the component states used in [Smartsoft](http://servicerobotik-ulm.de/drupal/?q=node/46).
    *   **Modeling of error propagation and severity within subsystems**
        *   Describe causal dependencies between components of a (sub)system with regard to errors. (Sources of inspiration are Fault-Tree Analysis and various academic works.)
    *   **Integrate component modes/states with component-specific set of parameters into one model**
        *   Use of parameter model in style of dynamic reconfigure of ROS1.
        *   However, this would introduce a dependency between component lifecycle and component parameters?
    *   **Timing/causality-aware switching between modes**
        *   Two simple patterns in first step: (1.) Reconfigure components of a (sub)system sequentially according to a given list or (2.) reconfigure them simultaneously.

1.  Implementation
    *   **Modeling of system, sub-system and component modes**
        *   Strive for lightweight approach by a simple configuration file or even use of C++ API.
        *   Should be well manageable (diffable, mergeable) with Git.
    *   **API primitives for defining preconditions and invariants with regard to system configuration**

1.  Runtime/Execution
    *   **Distributed system state across multiple computing devices (uCs, uPs)**
        *   The envisioned approach shall support the communication of the modes/states of subsystems or components located on different computing devices to always obtain a consistent system view.
        *   It shall also consider that a computing device may reconfigure its sub-systems or components (in a limited scope) independent of the sub-systems and components other computing devices.

## System Error and Contingency Diagnosis

The envisioned approach shall also provide API primitives for detection of typical system errors and contingencies.

1.  **Monitors for communication layers**
    *   Check for receive rates, latencies, ...

1.  **Operating system monitors**

1.  **Templates and APIs for hardware monitors**
    *   Basic blocks for GPIOs (voltage, current)
    *   Basic blocks for connected HW modules (via SPI, I²C)

1.  **Templates and APIs for monitoring of functional properties**
    *   No need of status messages here, but use of function-oriented topics
    *   Building blocks/patterns for analysis of time-series (with a sliding window) for bounds, for delays, ...
