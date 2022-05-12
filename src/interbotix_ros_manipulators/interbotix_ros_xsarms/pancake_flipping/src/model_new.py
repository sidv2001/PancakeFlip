import argparse
from cmath import inf
import os
import optuna
from optuna.integration import PyTorchLightningPruningCallback
from packaging import version
import pytorch_lightning as pl
import torch
from torch import nn
from torch import optim
import torch.nn.functional as F
from torch.utils.data import DataLoader
from torch.utils.data import random_split
from torchvision import datasets
from torchvision import transforms
import dataset_maker as dataset_maker
from pytorch_lightning.callbacks import ModelCheckpoint


mse_loss = nn.MSELoss(reduction = 'mean')
BATCHSIZE = 128
CLASSES = 6
EPOCHS = 150
dataset = "datasets"


class RobotNet(nn.Module):
    def __init__(self, dropout, output_dims):
        super().__init__()
        layers = []

        input_dim = 16
        for output_dim in output_dims:
            layers.append(nn.Linear(input_dim, output_dim))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(dropout))
            input_dim = output_dim

        layers.append(nn.Linear(input_dim, CLASSES))

        self.layers = nn.Sequential(*layers)

    def forward(self, data):
        logits = self.layers(data)
        return logits

class RoboticRegression(pl.LightningModule):
    def __init__(self, dropout, output_dims, learning_rate):
        super().__init__()
        self.save_hyperparameters()
        self.learning_rate = learning_rate
        self.model = RobotNet(dropout, output_dims)
    
    def forward(self, x):
        return self.model(x)

    def training_step(self, batch, batch_idx):
        data, target = batch
        output = self(data)
        loss = F.mse_loss(output, target)
        self.log("train_loss", loss)
        return loss

    def validation_step(self, batch, batch_idx):
        data, target = batch
        output = self(data)
        loss = F.mse_loss(output,target)
        self.log("val_loss", loss)

    def configure_optimizers(self):
        return optim.Adam(self.parameters(), lr=self.learning_rate)

class RoboticsDataModule(pl.LightningDataModule):
    def __init__(self, dataset, val_dataset, batch_size):
        super().__init__()
        self.dataset = dataset
        self.batch_size = batch_size
        self.val_dataset = val_dataset

    def train_dataloader(self):
        train_dataset = dataset_maker_new.get_dataset(self.dataset)
        train_loader = DataLoader(dataset = train_dataset, batch_size = 128)
        return train_loader
        
    def val_dataloader(self):
        validation_dataset = dataset_maker_new.get_dataset(self.val_dataset)
        validation_loader = DataLoader(dataset = validation_dataset, batch_size = 128)
        return validation_loader
    
    def test_dataloader(self):
        test_dataset = dataset_maker_new.get_dataset(self.val_dataset)
        test_loader = DataLoader(dataset = test_dataset, batch_size = 128)
        return test_loader


def objective(trial):
    script_dir = os.path.dirname(os.path.realpath(__file__))
    n_layers = trial.suggest_int("n_layers", 3, 12)
    dropout = trial.suggest_float("dropout", 0, 0.3)
    output_dims = [
        trial.suggest_int("n_units_l{}".format(i), 20, 300, log=True) for i in range(n_layers)
    ]

    model = RoboticRegression(dropout, output_dims, (10**(-4)))
    datamodule = RoboticsDataModule("csv", "csv", 128)
    checkpoint_callback = ModelCheckpoint(
    monitor="val_loss",
    dirpath=script_dir,
    filename="robot-{epoch:02d}-{val_loss:.2f}-{train_loss:.2f}",
    save_top_k=1,
    mode="min")
    trainer = pl.Trainer(
        logger=True,
        max_epochs=EPOCHS,
        gpus=1 if torch.cuda.is_available() else None,
        callbacks=[PyTorchLightningPruningCallback(trial, monitor="val_loss"), checkpoint_callback],
        auto_lr_find=False
    )
    hyperparameters = dict(n_layers=n_layers, dropout=dropout, output_dims=output_dims)
    trainer.logger.log_hyperparams(hyperparameters)
    trainer.fit(model, datamodule=datamodule)

    return trainer.callback_metrics["val_loss"].item()


def optuna_opt():
    parser = argparse.ArgumentParser(description="PyTorch Lightning example.")
    parser.add_argument(
        "--pruning",
        "-p",
        action="store_true",
        help="Activate the pruning feature. `MedianPruner` stops unpromising "
        "trials at the early stages of training.",
    )
    args = parser.parse_args()

    pruner = (
        optuna.pruners.MedianPruner() if args.pruning else optuna.pruners.NopPruner()
    )

    study = optuna.create_study(direction="minimize", pruner=pruner)
    study.optimize(objective, n_trials=30, timeout=1800)

    print("Number of finished trials: {}".format(len(study.trials)))

    print("Best trial:")
    trial = study.best_trial

    print("  Value: {}".format(trial.value))

    print("  Params: ")
    for key, value in trial.params.items():
        print("    {}: {}".format(key, value))

if __name__ == "__main__":
    optuna_opt()