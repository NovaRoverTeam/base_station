from flask import Blueprint

# Creates set of blueprints based on content of the folder to be 
# imported into the main Flask App

routes = Blueprint('routes', __name__)
from .home import * # Home Page
from .control import * # Control Page
from .radio import * # Radio Page
from .surveillance import * # Camera Page
from .arm import * # Arm Page
