import glob
import os
from rosbags import convert
from pathlib import Path
import traceback
import time
import sys
import logging

# Configure logging to save progress to root of destination
logFilePath = '/rosbags2/conversion-log.txt'
logging.basicConfig(filename=logFilePath, level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

logging.info("Starting conversion")
srcRootPath = "/rosbags1/**/*.bag"
dstRootPath = "/rosbags2/"

# convert in order of size
allPaths = sorted(glob.glob(srcRootPath, recursive=True), key=os.path.getsize)
numProcessed = 0
numFailed = 0
for src in allPaths:
    size = os.path.getsize(src) >> 20
    startTime = time.time()

    dest = Path(dstRootPath + src.split("rosbags/")[1].rstrip(".bag"))
    logging.info(dest)

    # create the directory if not exists
    # destDir.mkdir(parents=True, exist_ok=True)
    try:
        convert.convert(Path(src), dest)

    # ConverterError can be thrown if converted rosbag already exists
    except convert.converter.ConverterError as e:
        logging.error(traceback.format_exc())
        logging.error(f"Encountered exception converting {src}, continuing...")
        numFailed += 1

    diffTime = time.time()-startTime
    numProcessed += 1
    logging.info(f"{numProcessed}/{len(allPaths)}: Converted {size} MB in {diffTime}s to {dest}")
    logging.info(f"Number of successful conversions: {numProcessed-numFailed} out of {numProcessed}")
