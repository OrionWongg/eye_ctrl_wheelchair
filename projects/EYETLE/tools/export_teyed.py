"""Export official Keras weights through the torch backend; verify ONNX parity."""
import os
os.environ['KERAS_BACKEND']='torch'
from pathlib import Path
import json
import keras
import numpy as np
import torch
import onnxruntime as ort

ROOT=Path(__file__).resolve().parents[1]/'models/teyed'

def main():
    torch.set_num_threads(4)
    reports={}
    for name,filename in [('teyed','teyed.keras'),('meye','meye.h5')]:
        model=keras.models.load_model(ROOT/filename,compile=False)
        if name=='meye':
            model=keras.Model(model.inputs,model.outputs[1])
        shape=tuple(model.input_shape[1:])
        class Export(torch.nn.Module):
            def __init__(self,net):
                super().__init__();self.net=net
            def forward(self,x):
                return self.net(x.permute(0,2,3,1),training=False)
        wrapper=Export(model).eval()
        sample=torch.from_numpy(np.random.default_rng(42).random((1,shape[2],shape[0],shape[1])).astype('float32'))
        path=ROOT/(name+'.onnx')
        with torch.no_grad():reference=wrapper(sample).numpy()
        torch.onnx.export(wrapper,sample,str(path),input_names=['eye'],output_names=['segmentation'],opset_version=17,dynamo=False)
        session=ort.InferenceSession(str(path),providers=['CPUExecutionProvider'])
        actual=session.run(None,{'eye':sample.numpy()})[0]
        np.testing.assert_allclose(actual,reference,atol=1e-4,rtol=1e-4)
        reports[name]=dict(input=list(sample.shape),output=list(actual.shape),max_error=float(np.max(np.abs(actual-reference))))
        print(name,reports[name],flush=True)
    (ROOT/'export-validation.json').write_text(json.dumps(reports,indent=2)+'\n')

if __name__=='__main__':main()
