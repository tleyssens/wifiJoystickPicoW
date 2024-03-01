import zitmaaier

print('start testzitmaaier')
zitmaaier = zitmaaier.Zitmaaier()
print('zitmaaier aangemaakt', dir(zitmaaier))
zitmaaier.rijden(0,0,0,1)
while True:
    zitmaaier.process()