from django.shortcuts import render
from .models import Arduino_data
from .utils import get_plot

# Create your views here.
def index(request):
    qs = Arduino_data.objects.all()
    x = [x.time for x in qs]
    y = [y.temperature for y in qs]
    chart = get_plot(x, y)

    # x_pid = [x_pid.time for x_pid in qs]
    # y_pid = [y_pid.pid for y_pid in qs]
    # chart_pid = get_plot(x_pid, y_pid)
    return render(request, 'data_viewer.html', {'chart': chart})
