import zitmaaier

print('start testzitmaaier')
zitmaaier = zitmaaier.Zitmaaier()
print('zitmaaier aangemaakt', dir(zitmaaier))
zitmaaier.rijden(1,1,1,1)
while True:
    zitmaaier.process()