import gurobipy as gp  # Gurobi library for optimization
from gurobipy import GRB  # Gurobi constants (like GRB.BINARY)
import csv  # To save results in CSV files

# Grid size: 5 rows and 8 columns
rows = 5
cols = 8

# Create a list of all grid positions
cells = [(i, j) for i in range(rows) for j in range(cols)]

# Components: FATO, Stand, Taxiway, Terminal
components = ['F', 'S', 'T', 'P']

# Cost of placing each component
cost = {'F': 40000, 'S': 46000, 'T': 40000, 'P': 300000}

# Maximum capacity per component type
fato_capacity = 30
stand_capacity = 6

# Total budget
budget = 2000000

# Big constant for constraints
M = 1000

# Start building the optimization model
model = gp.Model("Vertiport_Design")

# Variables: where to place components
place = model.addVars([(i, j, c) for i, j in cells for c in components], vtype=GRB.BINARY)

# Variable: total number of operations we want to maximize
operations = model.addVar(vtype=GRB.CONTINUOUS)

# Flow-related variables to ensure connectivity
reachable = model.addVars(cells, vtype=GRB.BINARY)
flow_balance = model.addVars(cells, vtype=GRB.CONTINUOUS)

# Allow flow only between neighboring cells
flow = model.addVars([(a, b) for a in cells for b in cells if abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1], vtype=GRB.CONTINUOUS)

# Objective: maximize operations
model.setObjective(operations, GRB.MAXIMIZE)

# Constraint: operations cannot exceed FATO or Stand capacity
model.addConstr(operations <= (fato_capacity / 2) * gp.quicksum(place[i[0], i[1], 'F'] for i in cells))
model.addConstr(operations <= stand_capacity * gp.quicksum(place[i[0], i[1], 'S'] for i in cells))

# Constraint: total cost should be within the budget
model.addConstr(gp.quicksum(cost[c] * place[i[0], i[1], c] for i in cells for c in components) <= budget)

# Constraint: only one component per cell
for i in cells:
    model.addConstr(gp.quicksum(place[i[0], i[1], c] for c in components) <= 1)

# Function to find neighbor cells (used in taxiway connections)
def get_neighbors(i):
    x, y = i
    neighbors = []
    if x > 0: neighbors.append((x - 1, y))
    if x < rows - 1: neighbors.append((x + 1, y))
    if y > 0: neighbors.append((x, y - 1))
    if y < cols - 1: neighbors.append((x, y + 1))
    return neighbors

# Constraint: F, S, and P must be next to at least one Taxiway (T)
for i in cells:
    neighbors = get_neighbors(i)
    model.addConstr(gp.quicksum(place[n[0], n[1], 'T'] for n in neighbors) >= place[i[0], i[1], 'F'])
    model.addConstr(gp.quicksum(place[n[0], n[1], 'T'] for n in neighbors) >= place[i[0], i[1], 'S'])
    model.addConstr(gp.quicksum(place[n[0], n[1], 'T'] for n in neighbors) >= place[i[0], i[1], 'P'])

# Constraint: no FATO in middle or too close to others
for i in cells:
    if 0 < i[0] < rows - 1 and 0 < i[1] < cols - 1:
        model.addConstr(place[i[0], i[1], 'F'] == 0)

    close_cells = [(i[0] + dx, i[1] + dy) for dx in [-2,-1,0,1,2] for dy in [-2,-1,0,1,2]
                   if abs(dx) + abs(dy) <= 2 and 0 <= i[0] + dx < rows and 0 <= i[1] + dy < cols and (dx != 0 or dy != 0)]
    for j in close_cells:
        model.addConstr(place[j[0], j[1], 'F'] <= (1 - place[i[0], i[1], 'F']) * M)

# Start flow from top-left corner
source = (0, 0)
model.addConstr(reachable[source] == 1)

# Flow balance constraints for connectivity
for i in cells:
    in_flow = gp.quicksum(flow[j, i] for j in cells if (j, i) in flow)
    out_flow = gp.quicksum(flow[i, j] for j in cells if (i, j) in flow)
    model.addConstr(flow_balance[i] == out_flow - in_flow)
    model.addConstr(flow_balance[i] <= M * reachable[i])
    model.addConstr(flow_balance[i] >= -M * reachable[i])

# Flow only allowed through taxiways
for i, j in flow:
    model.addConstr(flow[i, j] <= M * place[i[0], i[1], 'T'])

# Solve the model
model.optimize()

# If a solution is found, print and save the results
if model.status == GRB.OPTIMAL:
    print("\nBest number of operations:", operations.X)
    count = {'F': 0, 'S': 0, 'T': 0, 'P': 0}
    layout = [['-' for _ in range(cols)] for _ in range(rows)]

    for i in cells:
        for c in components:
            if place[i[0], i[1], c].X > 0.5:
                layout[i[0]][i[1]] = c
                count[c] += 1

    print("\nGrid Layout:")
    for row in layout:
        print(' '.join(row))

    print("\nComponent Counts:")
    for c in components:
        print(f"{c}: {count[c]}")

    used_money = sum(cost[c] * count[c] for c in components)
    print(f"\nBudget Used: {used_money} out of {budget}")

    # Save the layout as CSV
    with open("layout.csv", "w", newline='') as file:
        writer = csv.writer(file)
        writer.writerow([f"Grid Layout ({rows}x{cols})"])
        for row in layout:
            writer.writerow(row)

    # Save summary as CSV
    with open("summary.csv", "w", newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Component", "Count"])
        for c in components:
            writer.writerow([c, count[c]])
        writer.writerow([])
        writer.writerow(["Total Operations", operations.X])
        writer.writerow(["Budget Used", used_money])
        writer.writerow(["Budget Limit", budget])
