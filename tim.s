.section .data

.global ground_tim
.type ground_tim, @object

.global goblin_tim
.type goblin_tim, @object

ground_tim:
	.incbin "textures/GROUND.TIM"

goblin_tim:
	.incbin "textures/GOBLIN.TIM"
