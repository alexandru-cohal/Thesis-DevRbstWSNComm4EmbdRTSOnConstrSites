# Thesis-DevRbstWSNComm4EmbdRTSOnConstrSites
### The thesis developed within the *EMECS* Masters

- **Date**: November 2018
- **Purpose**: The purpose of my Master's thesis was to develop a Communication Protocol Stack for a CC1350-based Wireless Sensor Network running on a Real-Time Operating System (TI-RTOS) for digitalizing a construction site (i.e. tracking the equipments, materials and workers and monitoring the statuses of the machines and
tools, the health of the workers and the environment)
- **Programming Language**: C
- **Team**: Individual project
- **Approach**:
  - Based on the targeted domain, several requirements had to be defined, considering the general properties of a network and the capabilities of the used microcontroller (i.e. *Texas Instruments CC1350*) 
  - Overviews and comparisons were realized for different communication protocols and standards used in IoT, application layer protocols, spectrum access methods and security solutions
  - A combination between the *IEEE 802.15.4 e/g standard*, *TI 15.4 Stack* and *MQTT-SN application layer* for the *Sub-1 GHz* band and *BLE* for the *2.4 GHz* band were considered to be the most suited core choices for the stack
  - Based on all the choices, a complete Communication Protocol Stack was assembled and described and an architecture of the whole network was proposed
  - Laboratory and field tests were performed for a basic version of this stack, considering the use case of a highway construction site
  - The results obtained (i.e. the average number of received, lost and duplicate packets for different configurations) were analyzed
  - For more information about the solution, architecture, implementation, results, conclusions and improvements see [this document](documentation/ThesisDocumentation-DevRbstWSNComm4EmbdRTSOnConstrSites.pdf) and [this presentation](documentation/ThesisPresentation-DevRbstWSNComm4EmbdRTSOnConstrSites.pdf)
