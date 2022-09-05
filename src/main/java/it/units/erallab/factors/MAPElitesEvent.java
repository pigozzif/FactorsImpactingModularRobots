package it.units.erallab.factors;

import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.order.PartiallyOrderedCollection;

import java.io.Serializable;
import java.util.HashMap;
import java.util.Map;


public class MAPElitesEvent<G, S, F> implements Serializable {

  private final Evolver.State state;
  private final PartiallyOrderedCollection<Individual<G, S, F>> orderedPopulation;
  private final PartiallyOrderedCollection<Individual<G, S, F>> serializationPop;
  private final PartiallyOrderedCollection<Individual<G, S, F>> removedPop;
  private final Map<String, Object> attributes;


  public MAPElitesEvent(Evolver.State state, PartiallyOrderedCollection<Individual<G, S, F>> orderedPopulation, PartiallyOrderedCollection<Individual<G, S, F>> serializationPop, PartiallyOrderedCollection<Individual<G, S, F>> removedPop) {
    this.state = state.copy();
    this.orderedPopulation = orderedPopulation;
    this.serializationPop = serializationPop;
    this.removedPop = removedPop;
    attributes = new HashMap<>();
  }

  public MAPElitesEvent(Evolver.State state, PartiallyOrderedCollection<Individual<G, S, F>> orderedPopulation){
    this(state,orderedPopulation,null, null);
  }

  public Evolver.State getState() {
    return state;
  }

  public PartiallyOrderedCollection<Individual<G, S, F>> getOrderedPopulation() {
    return orderedPopulation;
  }

  public Map<String, Object> getAttributes() {
    return attributes;
  }

  public PartiallyOrderedCollection<Individual<G, S, F>> getSerializationPop(){
    return this.serializationPop;
  }
  public PartiallyOrderedCollection<Individual<G, S, F>> getRemovedPop(){
    return this.removedPop;
  }

}
