from django.db import models

# Create your models here.

class Arduino_data(models.Model):
    temperature = models.FloatField()
    fanspeed = models.FloatField()
    pid = models.FloatField()
    time = models.DateTimeField(auto_now_add=True)