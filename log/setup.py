import logging
import os
from datetime import datetime

log_dir = os.path.dirname(os.path.abspath(__file__)) + '/'
file_name = 'cloud_MAPF_'
file_name += datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
file_name += '.log'

logger = logging.getLogger(__name__)

# formatter = logging.Formatter('[%(asctime)s] [%(filename)s:%(lineno)s] %(message)s')
formatter = logging.Formatter('[%(asctime)s] %(message)s')

streamHandler = logging.StreamHandler()
fileHandler = logging.FileHandler(log_dir+file_name, 'w')

streamHandler.setFormatter(formatter)
fileHandler.setFormatter(formatter)

logger.addHandler(streamHandler)
logger.addHandler(fileHandler)

logger.setLevel(level=logging.INFO)