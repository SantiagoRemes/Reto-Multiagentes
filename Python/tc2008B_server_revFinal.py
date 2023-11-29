from http.server import BaseHTTPRequestHandler, HTTPServer
import logging
import json
from model import Bodega

# Here, you should have the model state or data that you want to send to Unity.

# Here empieza el server
class Server(BaseHTTPRequestHandler):
    
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')  # Change content type to application/json
        self.end_headers()
        
    def do_GET(self):
        logging.info("GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers))
        self._set_response()
    

        # Extracting query parameters from the URL
        params = {}
        if '?' in self.path:
            query_string = self.path.split('?', 1)[1]
            params = dict(qc.split("=") for qc in query_string.split("&"))

        # Setting default values if parameters are not provided in the request
        M = int(params.get('M', 36))
        N = int(params.get('N', 48))
        num_agentes = int(params.get('num_agentes', 6))
        steps_per_package_generation = int(params.get('steps_per_package_generation', 5))
        steps_create_order = int(params.get('steps_create_order', 10))
        num_steps = int(params.get('num_steps', 200))

        # Create a new instance of the Bodega class with the specified parameters
        model = Bodega(M=M, N=N, steps_per_package_generation=steps_per_package_generation, steps_create_order=steps_create_order, num_agentes=num_agentes, modo_pos_inicial='Aleatoria')

        # Additional steps to run the model
        for _ in range(num_steps):
            print(_)
            model.step()

        # Assuming get_model_state() is a method in your Bodega class that returns the model state
        model_state = model.generate_json()
        
        # Convert the model state to JSON
        response_data = json.dumps(model_state)

        self.wfile.write(response_data.encode('utf-8'))

def run(server_class=HTTPServer, handler_class=Server, port=8585):
    logging.basicConfig(level=logging.INFO)
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    logging.info("Starting httpd...\n")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    logging.info("Stopping httpd...\n")

if __name__ == '__main__':
    from sys import argv
    
    if len(argv) == 2:
        run(port=int(argv[1]))
    else:
        run()
