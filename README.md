This is a stand alone implementation and test of the plastic terramechanics particle system.
To visualize the simulation the OpenGL is used:

- build with 'make'
- run with '/terra'
- for test simulation with virtual position behavior
  . option 'k' used for lateral test simulation.
  . option 'j' used for virtical test simulation.

- for test simulation with real position behavior from real experiment
  . in this case we use the experiment data to control the contact object position behavior (Lateral: 2011_04_05_11_50_22log.csv and Virtical: 2011_05_04_14_12_14log.csv)
  . option 'K' used for lateral test simulation.
  . option 'J' used for virtical test simulation.

- Example of lateral test
  . option 'K' used for simulation with position behavior from real experiment.
  . cell setup:  cell numbers (200, 10, 30) , start(-200, -20, -30), end(200, 0,30)
  . simulation result will be printed to xx.csv

- Example of virtical test
  . option 'j used for sim.
  . cell setup:  cell numbers (200, 20, 30) , start(-200, -40, -30), end(200, 0,30)
  . simulation result will be printed to xx.csv
  
