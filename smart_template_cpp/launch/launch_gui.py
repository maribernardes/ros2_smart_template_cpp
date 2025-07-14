#!/usr/bin/env python3

import subprocess
import sys
import os
import time

def main():
    # Give a moment for other nodes to start
    print("Waiting 2 seconds for other nodes to start...")
    time.sleep(2)
    
    # Launch RQT with the smart_template_gui plugin
    print("Launching SmartTemplate GUI...")
    try:
        subprocess.run(['rqt', '--standalone', 'smart_template_gui'], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error launching GUI: {e}")
        return 1
    except KeyboardInterrupt:
        print("GUI launch interrupted")
        return 0
    
    return 0

if __name__ == '__main__':
    sys.exit(main())