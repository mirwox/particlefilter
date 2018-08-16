# Como acelerar (um pouco) o loop crítico dos LIDAR


Crie matrizes que vão ser reutilizadas pelo *inspercles*:

	retorno_lidar_robo = np.copy(lidar_map)
	retorno_lidar_particulas = np.copy(lidar_map)

Passe referências a estas matrizes:

**Para o robô:**

Use o retorno_lidar_robo para evitar que o lidar_map seja sempre recriado

	z_real, lidar_map = inspercles.nb_simulate_lidar(pose, angles, np_image, retorno = retorno_lidar_robo)

**Para as partículas:**


Use a flag output_image=False para evitar ter que limpar uma imagem do laser saindo de cada partícula a cada iteração

	for elemento in particles:
		z_barra, imagem = inspercles.nb_simulate_lidar([elemento.x,elemento.y,elemento.theta], angles, np_image, retorno = retorno_lidar_particulas, output_image = False)

