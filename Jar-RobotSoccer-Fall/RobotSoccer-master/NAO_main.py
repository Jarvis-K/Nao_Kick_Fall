# Main class

import NAO_config
import NAO_logger
import NAO_controller

controller = None


def start(self):
    ###
    # Summary: it starts the logger, the config and the controller
    # Parameters: none
    # Return: --
    ###

    logger = NAO_logger.Logger()
    config = NAO_config.Config(logger)



    global controller
    self.controller = NAO_controller.Controller(logger, config)


def start2(self):
    self.controller.start()
def stop():
    ###
    # Summary: it stops everything
    # Parameters: none
    # Return: --
    ###

    controller.isStop = True


if __name__ == "__main__":
    logger = NAO_logger.Logger()
    config = NAO_config.Config(logger)

    controller = NAO_controller.Controller(logger, config)
    controller.start()