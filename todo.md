
# TO-DO:

  ### CORE-DMA:
  - Edit names of config parameters to be more descriptive.;
  - Go through functions and ensure that writeback is properly updated
    in all -> especially (unlink) and (move constructor) 
    for TransferDescriptor struct in core_dma.
  - Handle case where transfer hit invalid descriptr (due to current editing
    of it's properties) -> need to revalidate descriptor & sync new settings