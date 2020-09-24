const sinon = require('sinon');
const sinonChai = require('sinon-chai');
const chai = require('chai');
const Library = require('../index');

const { expect } = chai;

chai.use(sinonChai);

let library;

// Replace with actual tests
describe('Library', () => {
  beforeEach(() => library = new Library());

  it('should get the library\'s name', () => {
    const spy = sinon.spy(library, 'name');
    const name = library.name();
    expect(name).to.eql('Library');
    spy.restore();
    expect(spy).to.have.been.called;
  });
});
