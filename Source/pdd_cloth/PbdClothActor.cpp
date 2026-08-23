// Fill out your copyright notice in the Description page of Project Settings.


#include "PbdClothActor.h"

// Sets default values
APbdClothActor::APbdClothActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void APbdClothActor::BeginPlay()
{
	Super::BeginPlay();
	Solver.InitializeGrid(NumX, NumY, Spacing);
}

// Called every frame
void APbdClothActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

