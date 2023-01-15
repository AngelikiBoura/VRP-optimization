from Solver import *

m = Model()
m.BuildModel('Instance.txt')
s = Solver(m)
sol = s.solve()
sol.save('Solution.txt')
