# Load-flow-analysis
Solving the load flow problem using Guass-Seidel iterative method. Written in C++.

## 3 Types of Buses
1. **Slack bus** or also known as swing bus.
2. **PQ bus** or Load bus.
3. **PV bus** or Gen bus.

## Bus Input Data
Format for bus input data. Same is followed in the busInputData.csv file.
- All the data is in pu (per unit).
- '-' specifies that the data is unknown.

| Type of Bus | V       | δ degrees | Gen MW | Gen Mvar  | Load MW | Load Mvar  | Gen Mvar Max | Gen Mvar Min |
| ----------- | ------------- | --------- | -------- | -------- | ------- | ------- | ------ | -------- | 
| 1 | 1 | 0 | - |-| 0|0|-|-|
|  2 |- |-|0|0|8|2.8|-|-|
| 3|1.05  |-|5.2|-|0.8|0.4|4.0|-2.8|
| 2  | - |-|0|0|0|0| -|-|
| 2  | - |-|0|0|0|0| -|-|

## Line Input Data
- All the data is in pu (per unit).

| From bus| To bus       | R | X|  G  | B | Max MVA  | 
| -------- | ------- | ---- | ---- | -------- | ------- | ------- |
| 2 | 4 | 0.0090 | 0.100 |0| 1.72|12|
|  2 |5 |0.0045|0.050|0|0.88|12|
| 4|5  |0.00225|0.025|0|0.44|12|
| 1  | 5 |0.00150|0.02|0|0|6|
| 3  | 4 |0.00075|0.01|0|0|10|

## Steps used to solve the load flow problem
- Both the csv files are read and Y Bus matrix is constructed.
- Using the Y Bus matrix, iterative Guass-Seidel method is used to solve for voltages and δ.
- For PV bus, reactive power is calculated and if it exceeds its limits then the bus type is changed to PQ bus and new value of V is calculated for that bus.
- If the calculated value does not exceed its limits then only δ is computed for that bus.
- Finally P and Q are calculated for slack bus.
- Bus Data is updated and stored in output.csv.

