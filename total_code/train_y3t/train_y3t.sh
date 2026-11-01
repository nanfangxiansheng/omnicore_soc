cd /home/hcy/ziguang/omini/tools/y3t
/home/hcy/anaconda3/envs/gputorch/bin/python train_custom_y3t.py \
    --data /home/hcy/ziguang/ziguang2/data.yaml \
    --nc 5 --epochs 300 --imgsz 96 \
    --device 0 \
    --name y3t_ziguang2
