# ... System related packages
import os
import time
# ... File related packages
import pandas as pd
import csv
import bagpy
from bagpy import bagreader
# ... Math related packages
import math
import numpy as np
from scipy.ndimage.filters import gaussian_filter, median_filter
# ... Plot related packages
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import seaborn as sns

location = '/home/avl/ur_ws/src/apple_proxy/bag_files/'
bagfile = 'apple_proxy_pick15-0.bag'

b = bagreader(location + bagfile)

file = str(bagfile)
file = file.replace('.bag', '', 1)

# Get the list of topics available in the file
print(b.topic_table)

# --- Read each topic ---
# Note: If the csvs from bagfiles are already there, then there is no need to read bagfile, only csv.
# This is important because it consumes time (specially the ones sampled at 500Hz)
start_reading_topics = time.time()
print('Start reading topics at: ', start_reading_topics)