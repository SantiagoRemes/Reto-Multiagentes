from model import Bodega

# Create an instance of the Bodega model
model = Bodega(M=36, N=48, steps_per_package_generation = 5,steps_create_order = 10, num_agentes=6, modo_pos_inicial='Aleatoria')

# Run the model for a certain number of steps
num_steps = 200
for _ in range(num_steps):
    print(_)
    
    model.step()

model.generate_json()

# The model has run for the specified number of steps
