#!/usr/bin/env python3

import sys
import signal
from PyQt5.QtWidgets import QApplication
from rqt_gui.main import Main

def main():
    # Enable Ctrl+C to stop the application
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # Create QApplication
    app = QApplication(sys.argv)
    
    # Create Main instance
    main = Main()
    
    # Run the standalone GUI
    sys.exit(main.main(sys.argv, standalone='smart_template_cpp.src.smart_template_gui:SmartTemplateGUIPlugin'))

if __name__ == '__main__':
    main()