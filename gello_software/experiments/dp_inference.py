from diffusion_policy.wrap_policy import get_workspace, preprocess
import numpy as np
workspace = get_workspace()

## inference
### sample input (1 frame)
data = dict(joint_positions=np.random.randn(7), base_rgb=np.random.randn(480, 640, 3))    ## 1 frame
### sample input (2 frames)
data = [dict(joint_positions=np.random.randn(7), base_rgb=np.random.randn(480, 640, 3)), dict(joint_positions=np.random.randn(7), base_rgb=np.random.randn(480, 640, 3))]    ## 2 frames

data = preprocess(data)
pred_action = workspace.predict_action(data)
print(pred_action[0])