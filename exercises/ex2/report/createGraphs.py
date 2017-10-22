import matplotlib.pyplot as plt
import csv
import pylab 
x=[3,5,8,16,23]
y ={}

with open('error.csv', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')

    for row in reader:
        y[row[0]] = row[1:]


pylab.plot(x, y["Laplace (uniform)"], label="Laplace (uniform)")
pylab.plot(x, y["Laplace"], label="Laplace")
pylab.legend(loc='upper right')
pylab.ylabel('Error')
pylab.xlabel('Grid size')
pylab.savefig('images/laplace.png'
pylab.show()

pylab.plot(x, y["Poisson (uniform)"], label="Poisson (uniform)")
pylab.plot(x, y["Poisson"], label="Poisson")
pylab.legend(loc='upper right')
pylab.ylabel('Error')
pylab.xlabel('Grid size')
pylab.savefig('images/poisson.png')
pylab.show()